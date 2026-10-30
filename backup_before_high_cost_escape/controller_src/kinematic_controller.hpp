#ifndef PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_CONTROLLER_HPP_
#define PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "plan_c_kinematic_controller/kinematic_law.hpp"

namespace plan_c_kinematic_controller
{

class KinematicController : public nav2_core::Controller
{
public:
  KinematicController() = default;
  ~KinematicController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool cancel() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  /**
   * @brief Path-to-Reference Adapter: extracts continuous reference state (xr, yr, thetar, vr, omegar)
   * from the discrete NavFn geometric global path.
   * (Engineering integration layer for Nav2, not from the original paper).
   */
  bool extractReferenceState(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const nav_msgs::msg::Path & plan,
    Pose2D & out_ref_pose,
    double & out_v_r,
    double & out_omega_r);

  bool extractReferenceState(
    const geometry_msgs::msg::PoseStamped & current_pose,
    Pose2D & out_ref_pose,
    double & out_v_r,
    double & out_omega_r)
  {
    return extractReferenceState(current_pose, global_plan_, out_ref_pose, out_v_r, out_omega_r);
  }

  ControllerState getState() const { return state_; }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("KinematicController")};
  rclcpp::Clock::SharedPtr clock_;

  nav_msgs::msg::Path global_plan_;
  double speed_limit_{1.0};
  bool speed_limit_is_percentage_{false};

  // Controller parameters
  ControllerGains gains_;
  RobotParameters robot_params_;
  TerminalParams terminal_params_;
  AlignToPathParams align_params_;
  CollisionCheckParams collision_params_;
  EscapeRecoveryParams escape_params_;
  ControllerState state_{ControllerState::TRACK_PATH};

  EscapeCandidate selected_escape_;
  Pose2D escape_start_pose_{0.0, 0.0, 0.0};
  rclcpp::Time escape_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time escape_stop_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time wait_path_start_time_{0, 0, RCL_ROS_TIME};
  int recovery_attempt_count_{0};
  int blocked_counter_{0};

  double lookahead_dist_{0.15};
  double nominal_linear_vel_{0.15};
  double max_linear_vel_{0.20};
  double max_angular_vel_{0.80};
  double transform_tolerance_{0.2};

  // Debug Publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr ref_twist_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tracking_error_pub_;
};

}  // namespace plan_c_kinematic_controller

#endif  // PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_CONTROLLER_HPP_
