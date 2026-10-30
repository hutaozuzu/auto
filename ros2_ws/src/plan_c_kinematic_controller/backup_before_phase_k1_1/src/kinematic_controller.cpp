#include "plan_c_kinematic_controller/kinematic_controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include "tf2/utils.h"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plan_c_kinematic_controller
{

void KinematicController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Parent lifecycle node was deleted!");
  }

  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // Declare & load parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k1", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k2", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".k3", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".nominal_linear_vel", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_radius", rclcpp::ParameterValue(0.0325));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_track_half", rclcpp::ParameterValue(0.0550));

  node->get_parameter(plugin_name_ + ".k1", gains_.k1);
  node->get_parameter(plugin_name_ + ".k2", gains_.k2);
  node->get_parameter(plugin_name_ + ".k3", gains_.k3);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".nominal_linear_vel", nominal_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(plugin_name_ + ".wheel_radius", robot_params_.r);
  node->get_parameter(plugin_name_ + ".wheel_track_half", robot_params_.b);

  RCLCPP_INFO(
    logger_,
    "Configured KinematicController '%s' (Gains: k1=%.2f, k2=%.2f, k3=%.2f | Lookahead=%.2fm, NomLinear=%.2fm/s)",
    plugin_name_.c_str(), gains_.k1, gains_.k2, gains_.k3, lookahead_dist_, nominal_linear_vel_);

  // Create debug publishers
  ref_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    plugin_name_ + "/reference_pose", rclcpp::SystemDefaultsQoS());
  ref_twist_pub_ = node->create_publisher<geometry_msgs::msg::TwistStamped>(
    plugin_name_ + "/reference_twist", rclcpp::SystemDefaultsQoS());
  tracking_error_pub_ = node->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    plugin_name_ + "/tracking_error", rclcpp::SystemDefaultsQoS());
}

void KinematicController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up KinematicController '%s'", plugin_name_.c_str());
  ref_pose_pub_.reset();
  ref_twist_pub_.reset();
  tracking_error_pub_.reset();
}

void KinematicController::activate()
{
  RCLCPP_INFO(logger_, "Activating KinematicController '%s'", plugin_name_.c_str());
  ref_pose_pub_->on_activate();
  ref_twist_pub_->on_activate();
  tracking_error_pub_->on_activate();
}

void KinematicController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating KinematicController '%s'", plugin_name_.c_str());
  ref_pose_pub_->on_deactivate();
  ref_twist_pub_->on_deactivate();
  tracking_error_pub_->on_deactivate();
}

void KinematicController::reset()
{
  global_plan_.poses.clear();
}

bool KinematicController::cancel()
{
  return true;
}

void KinematicController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void KinematicController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

bool KinematicController::extractReferenceState(
  const geometry_msgs::msg::PoseStamped & current_pose,
  Pose2D & out_ref_pose,
  double & out_v_r,
  double & out_omega_r)
{
  if (global_plan_.poses.empty()) {
    return false;
  }

  double cur_x = current_pose.pose.position.x;
  double cur_y = current_pose.pose.position.y;

  // Step A: Find closest point index on global plan
  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i].pose.position.x - cur_x;
    double dy = global_plan_.poses[i].pose.position.y - cur_y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  // Step B: Walk forward along path by lookahead_dist arc-length
  size_t target_idx = closest_idx;
  double accumulated_dist = 0.0;
  for (size_t i = closest_idx; i + 1 < global_plan_.poses.size(); ++i) {
    double dx = global_plan_.poses[i + 1].pose.position.x - global_plan_.poses[i].pose.position.x;
    double dy = global_plan_.poses[i + 1].pose.position.y - global_plan_.poses[i].pose.position.y;
    accumulated_dist += std::hypot(dx, dy);
    target_idx = i + 1;
    if (accumulated_dist >= lookahead_dist_) {
      break;
    }
  }

  out_ref_pose.x = global_plan_.poses[target_idx].pose.position.x;
  out_ref_pose.y = global_plan_.poses[target_idx].pose.position.y;

  // Step C: Determine reference heading theta_r from local path tangent
  if (target_idx + 1 < global_plan_.poses.size()) {
    double dx = global_plan_.poses[target_idx + 1].pose.position.x - out_ref_pose.x;
    double dy = global_plan_.poses[target_idx + 1].pose.position.y - out_ref_pose.y;
    if (std::hypot(dx, dy) > 1e-4) {
      out_ref_pose.theta = std::atan2(dy, dx);
    } else {
      out_ref_pose.theta = tf2::getYaw(global_plan_.poses[target_idx].pose.orientation);
    }
  } else if (target_idx > 0) {
    double dx = out_ref_pose.x - global_plan_.poses[target_idx - 1].pose.position.x;
    double dy = out_ref_pose.y - global_plan_.poses[target_idx - 1].pose.position.y;
    if (std::hypot(dx, dy) > 1e-4) {
      out_ref_pose.theta = std::atan2(dy, dx);
    } else {
      out_ref_pose.theta = tf2::getYaw(global_plan_.poses[target_idx].pose.orientation);
    }
  } else {
    out_ref_pose.theta = tf2::getYaw(global_plan_.poses[target_idx].pose.orientation);
  }

  // Step D: Nominal linear reference speed v_r
  out_v_r = nominal_linear_vel_;

  // Step E: Estimate local curvature kappa ~= delta_heading / delta_arc_length
  double kappa = 0.0;
  if (target_idx + 1 < global_plan_.poses.size() && target_idx > 0) {
    double dx1 = out_ref_pose.x - global_plan_.poses[target_idx - 1].pose.position.x;
    double dy1 = out_ref_pose.y - global_plan_.poses[target_idx - 1].pose.position.y;
    double th1 = std::atan2(dy1, dx1);
    double ds1 = std::hypot(dx1, dy1);

    double dx2 = global_plan_.poses[target_idx + 1].pose.position.x - out_ref_pose.x;
    double dy2 = global_plan_.poses[target_idx + 1].pose.position.y - out_ref_pose.y;
    double th2 = std::atan2(dy2, dx2);
    double ds2 = std::hypot(dx2, dy2);

    double ds = 0.5 * (ds1 + ds2);
    if (ds > 1e-4) {
      double dth = KinematicLaw::normalizeAngle(th2 - th1);
      kappa = dth / ds;
    }
  }

  out_omega_r = out_v_r * kappa;
  return true;
}

geometry_msgs::msg::TwistStamped KinematicController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "KinematicController received empty global plan.");
    return cmd_vel;
  }

  // Check goal reaching
  if (goal_checker && goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // Extract Actual Robot Pose2D
  Pose2D actual_pose;
  actual_pose.x = pose.pose.position.x;
  actual_pose.y = pose.pose.position.y;
  actual_pose.theta = tf2::getYaw(pose.pose.orientation);

  // Extract Reference State (xr, yr, thetar, vr, omegar) via Adapter
  Pose2D ref_pose;
  double v_r = 0.0;
  double omega_r = 0.0;
  if (!extractReferenceState(pose, ref_pose, v_r, omega_r)) {
    return cmd_vel;
  }

  // Paper Eq. (13): Tracking Error Computation
  TrackingError err = KinematicLaw::computeTrackingError(actual_pose, ref_pose);

  // Paper Eq. (14): Whole-Robot Kinematic Control Law
  Velocity2D raw_cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains_);

  // Determine active speed limits
  double effective_max_v = max_linear_vel_;
  if (speed_limit_is_percentage_) {
    effective_max_v *= (speed_limit_ / 100.0);
  } else if (speed_limit_ < max_linear_vel_ && speed_limit_ > 0.0) {
    effective_max_v = speed_limit_;
  }

  // Velocity Saturation
  Velocity2D sat_cmd = KinematicLaw::applySaturation(raw_cmd, effective_max_v, max_angular_vel_);

  cmd_vel.twist.linear.x = sat_cmd.v;
  cmd_vel.twist.angular.z = sat_cmd.omega;

  // Publish Debug Topics
  if (ref_pose_pub_ && ref_pose_pub_->is_activated()) {
    geometry_msgs::msg::PoseStamped ref_msg;
    ref_msg.header = cmd_vel.header;
    ref_msg.pose.position.x = ref_pose.x;
    ref_msg.pose.position.y = ref_pose.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, ref_pose.theta);
    ref_msg.pose.orientation = tf2::toMsg(q);
    ref_pose_pub_->publish(ref_msg);
  }

  if (ref_twist_pub_ && ref_twist_pub_->is_activated()) {
    geometry_msgs::msg::TwistStamped ref_twist_msg;
    ref_twist_msg.header = cmd_vel.header;
    ref_twist_msg.twist.linear.x = v_r;
    ref_twist_msg.twist.angular.z = omega_r;
    ref_twist_pub_->publish(ref_twist_msg);
  }

  if (tracking_error_pub_ && tracking_error_pub_->is_activated()) {
    geometry_msgs::msg::Vector3Stamped err_msg;
    err_msg.header = cmd_vel.header;
    err_msg.vector.x = err.e1;
    err_msg.vector.y = err.e2;
    err_msg.vector.z = err.e3;
    tracking_error_pub_->publish(err_msg);
  }

  return cmd_vel;
}

}  // namespace plan_c_kinematic_controller

PLUGINLIB_EXPORT_CLASS(plan_c_kinematic_controller::KinematicController, nav2_core::Controller)
