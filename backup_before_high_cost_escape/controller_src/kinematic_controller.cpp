#include "plan_c_kinematic_controller/kinematic_controller.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
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

  // Terminal heading parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_k_theta", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_max_angular_vel", rclcpp::ParameterValue(0.80));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".terminal_min_angular_vel", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".xy_goal_tolerance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".yaw_goal_tolerance", rclcpp::ParameterValue(0.15));

  // Collision checking parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".collision_check_horizon", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".collision_check_dt", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".treat_unknown_as_unsafe", rclcpp::ParameterValue(false));

  // Backup & Rotate Recovery parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".backup_recovery_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".backup_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".backup_distance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_backup_distance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".backup_timeout", rclcpp::ParameterValue(8.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".rotate_speed", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wait_path_timeout", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_recovery_attempts", rclcpp::ParameterValue(3));

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

  node->get_parameter(plugin_name_ + ".terminal_k_theta", terminal_params_.k_theta);
  node->get_parameter(plugin_name_ + ".terminal_max_angular_vel", terminal_params_.max_angular_vel);
  node->get_parameter(plugin_name_ + ".terminal_min_angular_vel", terminal_params_.min_angular_vel);
  node->get_parameter(plugin_name_ + ".xy_goal_tolerance", terminal_params_.xy_goal_tolerance);
  node->get_parameter(plugin_name_ + ".yaw_goal_tolerance", terminal_params_.yaw_goal_tolerance);

  node->get_parameter(plugin_name_ + ".collision_check_horizon", collision_params_.horizon);
  node->get_parameter(plugin_name_ + ".collision_check_dt", collision_params_.dt);
  node->get_parameter(plugin_name_ + ".treat_unknown_as_unsafe", collision_params_.treat_unknown_as_unsafe);

  // Align To Path parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_to_path_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_enter_threshold", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_exit_threshold", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_k", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_omega_max", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_omega_min", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".align_linear_vel", rclcpp::ParameterValue(0.02));

  node->get_parameter(plugin_name_ + ".align_to_path_enabled", align_params_.enabled);
  node->get_parameter(plugin_name_ + ".align_enter_threshold", align_params_.enter_threshold);
  node->get_parameter(plugin_name_ + ".align_exit_threshold", align_params_.exit_threshold);
  node->get_parameter(plugin_name_ + ".align_k", align_params_.k_align);
  node->get_parameter(plugin_name_ + ".align_omega_max", align_params_.max_angular_vel);
  node->get_parameter(plugin_name_ + ".align_omega_min", align_params_.min_angular_vel);
  node->get_parameter(plugin_name_ + ".align_linear_vel", align_params_.linear_vel);

  // General Wall Escape Recovery parameters
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_recovery_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_linear_speed", rclcpp::ParameterValue(0.03));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_angular_speed", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_arc_angular_speed", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_distance", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_max_distance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_rotation_angle", rclcpp::ParameterValue(0.785));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_stop_duration", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_timeout", rclcpp::ParameterValue(4.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wait_path_timeout", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_escape_attempts", rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".escape_block_trigger_cycles", rclcpp::ParameterValue(5));

  node->get_parameter(plugin_name_ + ".escape_recovery_enabled", escape_params_.enabled);
  node->get_parameter(plugin_name_ + ".escape_linear_speed", escape_params_.linear_speed);
  node->get_parameter(plugin_name_ + ".escape_angular_speed", escape_params_.angular_speed);
  node->get_parameter(plugin_name_ + ".escape_arc_angular_speed", escape_params_.arc_angular_speed);
  node->get_parameter(plugin_name_ + ".escape_distance", escape_params_.escape_distance);
  node->get_parameter(plugin_name_ + ".escape_max_distance", escape_params_.max_escape_distance);
  node->get_parameter(plugin_name_ + ".escape_rotation_angle", escape_params_.rotation_angle);
  node->get_parameter(plugin_name_ + ".escape_stop_duration", escape_params_.stop_duration);
  node->get_parameter(plugin_name_ + ".escape_timeout", escape_params_.timeout);
  node->get_parameter(plugin_name_ + ".wait_path_timeout", escape_params_.wait_path_timeout);
  node->get_parameter(plugin_name_ + ".max_escape_attempts", escape_params_.max_attempts);
  node->get_parameter(plugin_name_ + ".escape_block_trigger_cycles", escape_params_.block_trigger_cycles);

  RCLCPP_INFO(
    logger_,
    "Configured KinematicController '%s' (Gains: k1=%.2f, k2=%.2f, k3=%.2f | EscapeRecovery: enabled=%d, v=%.2fm/s, d=%.2fm)",
    plugin_name_.c_str(), gains_.k1, gains_.k2, gains_.k3,
    escape_params_.enabled, escape_params_.linear_speed, escape_params_.escape_distance);

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
  state_ = ControllerState::TRACK_PATH;
  recovery_attempt_count_ = 0;
  blocked_counter_ = 0;
}

void KinematicController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating KinematicController '%s'", plugin_name_.c_str());
  ref_pose_pub_->on_deactivate();
  ref_twist_pub_->on_deactivate();
  tracking_error_pub_->on_deactivate();
  state_ = ControllerState::TRACK_PATH;
}

void KinematicController::reset()
{
  global_plan_.poses.clear();
  state_ = ControllerState::TRACK_PATH;
  recovery_attempt_count_ = 0;
  blocked_counter_ = 0;
}

bool KinematicController::cancel()
{
  state_ = ControllerState::TRACK_PATH;
  recovery_attempt_count_ = 0;
  blocked_counter_ = 0;
  return true;
}

void KinematicController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (state_ == ControllerState::WAIT_NEW_PATH) {
    RCLCPP_INFO(logger_, "Received fresh global path (%zu poses) in WAIT_NEW_PATH. Resuming TRACK_PATH!", path.poses.size());
    state_ = ControllerState::TRACK_PATH;
    blocked_counter_ = 0;
  }
}

void KinematicController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

bool KinematicController::extractReferenceState(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const nav_msgs::msg::Path & plan,
  Pose2D & out_ref_pose,
  double & out_v_r,
  double & out_omega_r)
{
  if (plan.poses.empty()) {
    return false;
  }

  double cur_x = current_pose.pose.position.x;
  double cur_y = current_pose.pose.position.y;

  // Step A: Find closest point index on path
  size_t closest_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();
  for (size_t i = 0; i < plan.poses.size(); ++i) {
    double dx = plan.poses[i].pose.position.x - cur_x;
    double dy = plan.poses[i].pose.position.y - cur_y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  // Step B: Walk forward along path by lookahead_dist arc-length
  size_t target_idx = closest_idx;
  double accumulated_dist = 0.0;
  for (size_t i = closest_idx; i + 1 < plan.poses.size(); ++i) {
    double dx = plan.poses[i + 1].pose.position.x - plan.poses[i].pose.position.x;
    double dy = plan.poses[i + 1].pose.position.y - plan.poses[i].pose.position.y;
    accumulated_dist += std::hypot(dx, dy);
    target_idx = i + 1;
    if (accumulated_dist >= lookahead_dist_) {
      break;
    }
  }

  out_ref_pose.x = plan.poses[target_idx].pose.position.x;
  out_ref_pose.y = plan.poses[target_idx].pose.position.y;

  // Step C: Determine reference heading theta_r from local path tangent
  if (target_idx + 1 < plan.poses.size()) {
    double dx = plan.poses[target_idx + 1].pose.position.x - out_ref_pose.x;
    double dy = plan.poses[target_idx + 1].pose.position.y - out_ref_pose.y;
    if (std::hypot(dx, dy) > 1e-4) {
      out_ref_pose.theta = std::atan2(dy, dx);
    } else {
      out_ref_pose.theta = tf2::getYaw(plan.poses[target_idx].pose.orientation);
    }
  } else if (target_idx > 0) {
    double dx = out_ref_pose.x - plan.poses[target_idx - 1].pose.position.x;
    double dy = out_ref_pose.y - plan.poses[target_idx - 1].pose.position.y;
    if (std::hypot(dx, dy) > 1e-4) {
      out_ref_pose.theta = std::atan2(dy, dx);
    } else {
      out_ref_pose.theta = tf2::getYaw(plan.poses[target_idx].pose.orientation);
    }
  } else {
    out_ref_pose.theta = tf2::getYaw(plan.poses[target_idx].pose.orientation);
  }

  // Step D: Nominal linear reference speed v_r
  out_v_r = nominal_linear_vel_;

  // Step E: Estimate local curvature kappa ~= delta_heading / delta_arc_length
  double kappa = 0.0;
  if (target_idx + 1 < plan.poses.size() && target_idx > 0) {
    double dx1 = out_ref_pose.x - plan.poses[target_idx - 1].pose.position.x;
    double dy1 = out_ref_pose.y - plan.poses[target_idx - 1].pose.position.y;
    double th1 = std::atan2(dy1, dx1);
    double ds1 = std::hypot(dx1, dy1);

    double dx2 = plan.poses[target_idx + 1].pose.position.x - out_ref_pose.x;
    double dy2 = plan.poses[target_idx + 1].pose.position.y - out_ref_pose.y;
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
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = pose.header.frame_id;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "KinematicController received empty global plan.");
    return cmd_vel;
  }

  // CRITICAL: Transform global_plan_ into the controller's working frame (pose.header.frame_id, e.g. "odom")
  nav_msgs::msg::Path transformed_plan;
  if (global_plan_.header.frame_id != pose.header.frame_id) {
    try {
      geometry_msgs::msg::TransformStamped tf_transform = tf_->lookupTransform(
        pose.header.frame_id,
        global_plan_.header.frame_id,
        tf2::TimePointZero,
        tf2::durationFromSec(transform_tolerance_));
      transformed_plan.header = pose.header;
      transformed_plan.poses.resize(global_plan_.poses.size());
      for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        tf2::doTransform(global_plan_.poses[i], transformed_plan.poses[i], tf_transform);
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "Failed to transform global plan from '%s' to '%s': %s",
        global_plan_.header.frame_id.c_str(), pose.header.frame_id.c_str(), ex.what());
      return cmd_vel;
    }
  } else {
    transformed_plan = global_plan_;
  }

  // Check goal reaching via Nav2 GoalChecker
  if (goal_checker && goal_checker->isGoalReached(pose.pose, transformed_plan.poses.back().pose, velocity)) {
    state_ = ControllerState::GOAL_REACHED;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // Extract Actual Robot Pose2D
  Pose2D actual_pose;
  actual_pose.x = pose.pose.position.x;
  actual_pose.y = pose.pose.position.y;
  actual_pose.theta = tf2::getYaw(pose.pose.orientation);

  // Check distance to terminal goal point in the SAME frame
  const auto & goal_pose_msg = transformed_plan.poses.back();
  double dx_goal = goal_pose_msg.pose.position.x - actual_pose.x;
  double dy_goal = goal_pose_msg.pose.position.y - actual_pose.y;
  double dist_to_goal = std::hypot(dx_goal, dy_goal);
  double goal_yaw = tf2::getYaw(goal_pose_msg.pose.orientation);

  Velocity2D raw_cmd{0.0, 0.0};

  // =============================================================
  // STATE: ESCAPE_SEARCH (Đánh giá các hướng thoát an toàn A..H)
  // =============================================================
  if (state_ == ControllerState::ESCAPE_SEARCH) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    std::vector<EscapeCandidate> all_candidates;
    Pose2D goal_2d{goal_pose_msg.pose.position.x, goal_pose_msg.pose.position.y, goal_yaw};

    bool found = KinematicLaw::selectBestEscapeDirection(
      actual_pose,
      goal_2d,
      costmap_ros_->getRobotFootprint(),
      costmap_ros_->getCostmap(),
      escape_params_,
      collision_params_,
      selected_escape_,
      all_candidates);

    if (found) {
      RCLCPP_INFO(
        logger_,
        "ESCAPE_SEARCH selected best safe maneuver: '%s' (v=%.2f, omega=%.2f, score=%.2f, max_cost=%.1f)",
        selected_escape_.name.c_str(), selected_escape_.v, selected_escape_.omega,
        selected_escape_.score, selected_escape_.max_cost);
      state_ = ControllerState::ESCAPE_EXECUTE;
      escape_start_pose_ = actual_pose;
      escape_start_time_ = clock_->now();
    } else {
      RCLCPP_WARN(logger_, "ESCAPE_SEARCH: ALL candidate escape motions hit obstacles! Entering BLOCKED_STOP.");
      state_ = ControllerState::BLOCKED_STOP;
    }
    return cmd_vel;
  }

  // =============================================================
  // STATE: ESCAPE_EXECUTE (Thực thi chuyển động thoát đã chọn)
  // =============================================================
  if (state_ == ControllerState::ESCAPE_EXECUTE) {
    double elapsed_sec = (clock_->now() - escape_start_time_).seconds();
    double dx = actual_pose.x - escape_start_pose_.x;
    double dy = actual_pose.y - escape_start_pose_.y;
    double disp = std::hypot(dx, dy);
    double dth = std::abs(KinematicLaw::normalizeAngle(actual_pose.theta - escape_start_pose_.theta));

    bool finished = false;
    if (std::abs(selected_escape_.v) > 1e-4) {
      if (disp >= selected_escape_.target_dist || disp >= escape_params_.max_escape_distance || elapsed_sec >= escape_params_.timeout) {
        finished = true;
      }
    } else {
      if (dth >= selected_escape_.target_angle || elapsed_sec >= escape_params_.timeout) {
        finished = true;
      }
    }

    if (finished) {
      RCLCPP_INFO(
        logger_,
        "ESCAPE_EXECUTE completed '%s' (disp=%.3fm, dth=%.1f deg, elapsed=%.2fs). Transitioning to ESCAPE_STOP.",
        selected_escape_.name.c_str(), disp, dth * 180.0 / M_PI, elapsed_sec);
      state_ = ControllerState::ESCAPE_STOP;
      escape_stop_start_time_ = clock_->now();
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      return cmd_vel;
    }

    // Dynamic continuous collision check during escape execution
    if (costmap_ros_) {
      bool is_safe = KinematicLaw::isTrajectorySafe(
        actual_pose.x,
        actual_pose.y,
        actual_pose.theta,
        selected_escape_.v,
        selected_escape_.omega,
        costmap_ros_->getRobotFootprint(),
        costmap_ros_->getCostmap(),
        collision_params_);

      if (!is_safe) {
        RCLCPP_WARN(
          logger_,
          "Obstacle detected during ESCAPE_EXECUTE ('%s')! Stopping immediately in ESCAPE_STOP.",
          selected_escape_.name.c_str());
        state_ = ControllerState::ESCAPE_STOP;
        escape_stop_start_time_ = clock_->now();
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = 0.0;
        return cmd_vel;
      }
    }

    cmd_vel.twist.linear.x = selected_escape_.v;
    cmd_vel.twist.angular.z = selected_escape_.omega;
    return cmd_vel;
  }

  // =============================================================
  // STATE: ESCAPE_STOP (Dừng 0.3s & Hủy tracking state cũ)
  // =============================================================
  if (state_ == ControllerState::ESCAPE_STOP) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    double elapsed_stop = (clock_->now() - escape_stop_start_time_).seconds();
    if (elapsed_stop >= escape_params_.stop_duration) {
      RCLCPP_INFO(logger_, "ESCAPE_STOP complete. Discarding old local plan and entering WAIT_NEW_PATH...");
      global_plan_.poses.clear();
      state_ = ControllerState::WAIT_NEW_PATH;
      wait_path_start_time_ = clock_->now();
    }
    return cmd_vel;
  }

  // =============================================================
  // STATE: WAIT_NEW_PATH (Dừng chờ Nav2 Planner cấp Global Path mới)
  // =============================================================
  if (state_ == ControllerState::WAIT_NEW_PATH) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;

    double elapsed_wait = (clock_->now() - wait_path_start_time_).seconds();

    // Nếu đã có global path mới và không rỗng
    if (!transformed_plan.poses.empty()) {
      Pose2D ref_pose;
      double v_r = 0.0, omega_r = 0.0;
      if (extractReferenceState(pose, transformed_plan, ref_pose, v_r, omega_r)) {
        bool is_fresh_safe = KinematicLaw::isTrajectorySafe(
          actual_pose.x, actual_pose.y, actual_pose.theta,
          nominal_linear_vel_, 0.0,
          costmap_ros_->getRobotFootprint(),
          costmap_ros_->getCostmap(),
          collision_params_);

        if (is_fresh_safe) {
          RCLCPP_INFO(logger_, "Fresh global path verified safe! Resuming TRACK_PATH.");
          state_ = ControllerState::TRACK_PATH;
          blocked_counter_ = 0;
          return cmd_vel;
        } else {
          RCLCPP_WARN(logger_, "Fresh global path is still blocked! Re-entering ESCAPE_SEARCH...");
          if (recovery_attempt_count_ < escape_params_.max_attempts) {
            recovery_attempt_count_++;
            state_ = ControllerState::ESCAPE_SEARCH;
          } else {
            RCLCPP_WARN(logger_, "Max recovery attempts reached (%d/%d)! BLOCKED_STOP.",
                        recovery_attempt_count_, escape_params_.max_attempts);
            state_ = ControllerState::BLOCKED_STOP;
          }
          return cmd_vel;
        }
      }
    }

    if (elapsed_wait >= escape_params_.wait_path_timeout) {
      RCLCPP_WARN(logger_, "Wait for new path timed out (%.1fs). Re-evaluating ESCAPE_SEARCH...", escape_params_.wait_path_timeout);
      if (recovery_attempt_count_ < escape_params_.max_attempts) {
        recovery_attempt_count_++;
        state_ = ControllerState::ESCAPE_SEARCH;
      } else {
        RCLCPP_WARN(logger_, "Max recovery attempts reached (%d/%d)! BLOCKED_STOP.",
                    recovery_attempt_count_, escape_params_.max_attempts);
        state_ = ControllerState::BLOCKED_STOP;
      }
    }
    return cmd_vel;
  }

  // =============================================================
  // STATE: BLOCKED_STOP Mode (Dừng an toàn)
  // =============================================================
  if (state_ == ControllerState::BLOCKED_STOP) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // Nếu plan rỗng (đang chờ cập nhật), đứng yên an toàn
  if (transformed_plan.poses.empty()) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  // State Machine: Terminal Mode vs ALIGN_TO_PATH vs TRACK_PATH
  if (dist_to_goal <= terminal_params_.xy_goal_tolerance) {
    // TERMINAL ROTATION MODE (Goal position reached, align with goal final orientation)
    raw_cmd.v = 0.0;
    raw_cmd.omega = KinematicLaw::computeTerminalRotation(
      actual_pose.theta, goal_yaw, terminal_params_, state_);
  } else {
    Pose2D ref_pose;
    double v_r = 0.0;
    double omega_r = 0.0;
    if (!extractReferenceState(pose, transformed_plan, ref_pose, v_r, omega_r)) {
      return cmd_vel;
    }

    // Determine heading toward target lookahead point from robot's current position
    double heading_to_target = std::atan2(ref_pose.y - actual_pose.y, ref_pose.x - actual_pose.x);
    double heading_err = KinematicLaw::normalizeAngle(heading_to_target - actual_pose.theta);

    // Check ALIGN_TO_PATH state transitions with hysteresis
    if (state_ == ControllerState::ALIGN_TO_PATH) {
      if (std::abs(heading_err) < align_params_.exit_threshold) {
        state_ = ControllerState::TRACK_PATH;
        RCLCPP_INFO(
          logger_,
          "Heading aligned to path (err=%.2f deg < exit threshold %.2f deg). Transitioning to TRACK_PATH.",
          heading_err * 180.0 / M_PI, align_params_.exit_threshold * 180.0 / M_PI);
      }
    } else if (align_params_.enabled && std::abs(heading_err) > align_params_.enter_threshold) {
      state_ = ControllerState::ALIGN_TO_PATH;
      RCLCPP_INFO(
        logger_,
        "Heading error to path is large (%.2f deg > enter threshold %.2f deg). Entering ALIGN_TO_PATH mode...",
        heading_err * 180.0 / M_PI, align_params_.enter_threshold * 180.0 / M_PI);
    }

    if (state_ == ControllerState::ALIGN_TO_PATH) {
      // Gentle forward rolling arc (v = align_linear_vel = 2 cm/s) to prevent wheel scrub slip and keep laser locked to wall
      raw_cmd.v = align_params_.linear_vel;
      double omega_cmd = std::clamp(
        align_params_.k_align * heading_err,
        -align_params_.max_angular_vel,
        align_params_.max_angular_vel);
      if (std::abs(omega_cmd) < align_params_.min_angular_vel) {
        omega_cmd = (heading_err >= 0.0 ? 1.0 : -1.0) * align_params_.min_angular_vel;
      }
      raw_cmd.omega = omega_cmd;
    } else {
      // TRACK_PATH MODE (Paper Eq. 13 - 14) with error-dependent speed modulation
      state_ = ControllerState::TRACK_PATH;
      TrackingError err = KinematicLaw::computeTrackingError(actual_pose, ref_pose);

      // Modulate reference speed based on tracking errors:
      // large |e3| (heading error) or |e2| (lateral offset) -> smoothly slow down to prevent corner cutting
      double speed_factor = std::max(0.2, std::cos(err.e3)) / (1.0 + 3.0 * std::abs(err.e2));
      double v_r_scaled = v_r * speed_factor;

      raw_cmd = KinematicLaw::computeControlLaw(err, v_r_scaled, omega_r, gains_);
    }

    // Publish Debug Topics in the matching frame
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
      TrackingError err = KinematicLaw::computeTrackingError(actual_pose, ref_pose);
      geometry_msgs::msg::Vector3Stamped err_msg;
      err_msg.header = cmd_vel.header;
      err_msg.vector.x = err.e1;
      err_msg.vector.y = err.e2;
      err_msg.vector.z = err.e3;
      tracking_error_pub_->publish(err_msg);
    }
  }

  // Determine active speed limits
  double effective_max_v = max_linear_vel_;
  if (speed_limit_is_percentage_) {
    effective_max_v *= (speed_limit_ / 100.0);
  } else if (speed_limit_ < max_linear_vel_ && speed_limit_ > 0.0) {
    effective_max_v = speed_limit_;
  }

  // Velocity Saturation
  Velocity2D sat_cmd = KinematicLaw::applySaturation(raw_cmd, effective_max_v, max_angular_vel_);

  // Forward Trajectory Collision Checking on Local Costmap
  if (costmap_ros_) {
    bool is_safe = KinematicLaw::isTrajectorySafe(
      actual_pose.x,
      actual_pose.y,
      actual_pose.theta,
      sat_cmd.v,
      sat_cmd.omega,
      costmap_ros_->getRobotFootprint(),
      costmap_ros_->getCostmap(),
      collision_params_);

    if (!is_safe) {
      blocked_counter_++;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = 0.0;

      // Khi bị chặn liên tục (blocked persistently >= block_trigger_cycles = 5 cycles ~= 0.25s)
      if (blocked_counter_ >= escape_params_.block_trigger_cycles) {
        if (escape_params_.enabled && recovery_attempt_count_ < escape_params_.max_attempts) {
          recovery_attempt_count_++;
          RCLCPP_INFO(
            logger_,
            "Robot BLOCKED PERSISTENTLY (%d cycles). Triggering general ESCAPE_SEARCH (attempt %d/%d)...",
            blocked_counter_, recovery_attempt_count_, escape_params_.max_attempts);
          state_ = ControllerState::ESCAPE_SEARCH;
        } else {
          RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Robot blocked persistently and recovery attempts exhausted (%d/%d)! BLOCKED_STOP.",
            recovery_attempt_count_, escape_params_.max_attempts);
          state_ = ControllerState::BLOCKED_STOP;
        }
      }
      return cmd_vel;
    } else {
      blocked_counter_ = 0;
    }
  }

  cmd_vel.twist.linear.x = sat_cmd.v;
  cmd_vel.twist.angular.z = sat_cmd.omega;

  return cmd_vel;
}

}  // namespace plan_c_kinematic_controller

PLUGINLIB_EXPORT_CLASS(plan_c_kinematic_controller::KinematicController, nav2_core::Controller)
