#ifndef PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_LAW_HPP_
#define PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_LAW_HPP_

#include <cmath>
#include <vector>
#include "geometry_msgs/msg/point.hpp"

namespace nav2_costmap_2d
{
class Costmap2D;
typedef std::vector<geometry_msgs::msg::Point> Footprint;
}

namespace plan_c_kinematic_controller
{

enum class ControllerState
{
  ALIGN_TO_PATH,
  TRACK_PATH,
  TERMINAL_ROTATE,
  GOAL_REACHED,
  ESCAPE_SEARCH,
  ESCAPE_EXECUTE,
  ESCAPE_STOP,
  WAIT_NEW_PATH,
  BLOCKED_STOP
};

struct AlignToPathParams
{
  bool enabled{true};
  double enter_threshold{0.35};  // rad (~20.0 deg)
  double exit_threshold{0.20};   // rad (~11.5 deg)
  double k_align{1.5};
  double max_angular_vel{0.60};  // rad/s (~34.4 deg/s)
  double min_angular_vel{0.50};  // rad/s (overcome 8.0 RPM ESP32 filter)
};

enum class EscapeMotionType
{
  FORWARD_LEFT_ARC,
  FORWARD_RIGHT_ARC,
  REVERSE_LEFT_ARC,
  REVERSE_RIGHT_ARC,
  STRAIGHT_REVERSE,
  ROTATE_LEFT,
  ROTATE_RIGHT,
  STRAIGHT_FORWARD
};

struct EscapeCandidate
{
  EscapeMotionType type{EscapeMotionType::STRAIGHT_REVERSE};
  std::string name{"straight_reverse"};
  double v{0.0};              // m/s
  double omega{0.0};          // rad/s
  double target_dist{0.10};   // m
  double target_angle{0.785}; // rad (~45 deg)
  bool is_safe{false};
  double max_cost{0.0};
  double min_clearance{1.0};
  double score{-1e9};
};

struct EscapeRecoveryParams
{
  bool enabled{true};
  double linear_speed{0.03};        // m/s (3 cm/s)
  double angular_speed{0.55};       // rad/s (~31.5 deg/s)
  double arc_angular_speed{0.30};   // rad/s for arcs
  double escape_distance{0.10};     // m (10 cm)
  double max_escape_distance{0.15}; // m (15 cm)
  double rotation_angle{0.785};     // rad (45 deg)
  double stop_duration{0.30};       // seconds stop before wait_new_path
  double timeout{4.0};              // seconds
  double wait_path_timeout{1.5};    // seconds
  int max_attempts{3};
  int block_trigger_cycles{5};      // ~0.25s at 20Hz
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct Velocity2D
{
  double v{0.0};
  double omega{0.0};
};

struct TrackingError
{
  double e1{0.0};  // Longitudinal error in robot coordinate frame
  double e2{0.0};  // Lateral error in robot coordinate frame
  double e3{0.0};  // Heading angle error normalized to [-pi, pi]
};

struct WheelRPM
{
  double left{0.0};
  double right{0.0};
};

struct ControllerGains
{
  double k1{1.0};
  double k2{2.0};
  double k3{1.0};
};

struct RobotParameters
{
  double r{0.0325};  // Wheel radius in meters (D = 0.065 m)
  double b{0.0550};  // Half wheel track separation in meters (L = 0.110 m)
};

struct TerminalParams
{
  double k_theta{1.5};
  double max_angular_vel{0.80};
  double min_angular_vel{0.0};
  double xy_goal_tolerance{0.15};
  double yaw_goal_tolerance{0.15};
};

struct CollisionCheckParams
{
  double horizon{0.8};             // Prediction horizon in seconds
  double dt{0.05};                 // Simulation step size in seconds
  bool treat_unknown_as_unsafe{false}; // Policy for costmap unknown cells
};

/**
 * @brief Pure mathematical implementation of the whole-robot nonholonomic kinematic tracking control law
 * combined with terminal orientation control and forward collision checking.
 */
class KinematicLaw
{
public:
  /**
   * @brief Normalize angle to [-pi, +pi]
   */
  static double normalizeAngle(double angle);

  /**
   * @brief Compute tracking error vector e = [e1, e2, e3]^T in robot body frame (Paper Eq. 13)
   * e1 =  cos(theta)*(x_r - x) + sin(theta)*(y_r - y)
   * e2 = -sin(theta)*(x_r - x) + cos(theta)*(y_r - y)
   * e3 = wrap_to_pi(theta_r - theta)
   */
  static TrackingError computeTrackingError(const Pose2D & actual, const Pose2D & reference);

  /**
   * @brief Compute whole-robot kinematic control law (Paper Eq. 14)
   * v_d     = v_r * cos(e3) + k1 * e1
   * omega_d = omega_r + k2 * v_r * e2 + k3 * v_r * sin(e3)
   */
  static Velocity2D computeControlLaw(
    const TrackingError & error,
    double v_r,
    double omega_r,
    const ControllerGains & gains);

  /**
   * @brief Terminal orientation controller: calculates in-place rotation when robot is within XY tolerance of goal.
   * (Engineering extension for Nav2 integration, not in the paper).
   */
  static double computeTerminalRotation(
    double current_yaw,
    double goal_yaw,
    const TerminalParams & params,
    ControllerState & out_state);

  /**
   * @brief Differential drive inverse kinematics (Paper Eq. 15 - 16)
   * omega_R = (v_d + b * omega_d) / r
   * omega_L = (v_d - b * omega_d) / r
   * RPM = omega_wheel * 60 / (2*pi)
   */
  static WheelRPM computeInverseKinematics(
    const Velocity2D & cmd,
    const RobotParameters & params = RobotParameters());

  /**
   * @brief Apply velocity saturation limits preserving signs
   */
  static Velocity2D applySaturation(
    const Velocity2D & cmd,
    double max_v,
    double max_omega);

  /**
   * @brief Forward simulates candidate command (v, omega) over horizon and tests footprint on local costmap
   */
  static bool isTrajectorySafe(
    double x0,
    double y0,
    double theta0,
    double v,
    double omega,
    const nav2_costmap_2d::Footprint & footprint,
    nav2_costmap_2d::Costmap2D * costmap,
    const CollisionCheckParams & params);

  /**
   * @brief Evaluates discrete escape directions (0, +-30, +-60, +-90, 180 deg) using full footprint sweeps on costmap.
   * Returns true if at least one safe escape trajectory is found.
   */
  static bool selectBestEscapeDirection(
    const Pose2D & robot_pose,
    const Pose2D & goal_pose,
    const nav2_costmap_2d::Footprint & footprint,
    nav2_costmap_2d::Costmap2D * costmap,
    const EscapeRecoveryParams & params,
    const CollisionCheckParams & collision_params,
    EscapeCandidate & out_best_candidate,
    std::vector<EscapeCandidate> & out_all_candidates);
};

}  // namespace plan_c_kinematic_controller

#endif  // PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_LAW_HPP_
