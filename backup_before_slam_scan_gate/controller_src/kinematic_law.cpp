#include "plan_c_kinematic_controller/kinematic_law.hpp"
#include <algorithm>
#include <cmath>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"

namespace plan_c_kinematic_controller
{

double KinematicLaw::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

TrackingError KinematicLaw::computeTrackingError(const Pose2D & actual, const Pose2D & reference)
{
  TrackingError err;
  double dx = reference.x - actual.x;
  double dy = reference.y - actual.y;
  double dtheta = reference.theta - actual.theta;

  double cos_th = std::cos(actual.theta);
  double sin_th = std::sin(actual.theta);

  // Paper Eq. (13)
  err.e1 = cos_th * dx + sin_th * dy;
  err.e2 = -sin_th * dx + cos_th * dy;
  err.e3 = normalizeAngle(dtheta);

  return err;
}

Velocity2D KinematicLaw::computeControlLaw(
  const TrackingError & error,
  double v_r,
  double omega_r,
  const ControllerGains & gains)
{
  Velocity2D cmd;

  // Paper Eq. (14)
  // v_d = v_r * cos(e3) + k1 * e1
  cmd.v = v_r * std::cos(error.e3) + gains.k1 * error.e1;

  // Dung sai vùng chết nhỏ (+-1.5cm cho sai số ngang, +-1.5 độ cho góc) để bám đường êm ái, triệt tiêu vi rung giật
  double effective_e2 = error.e2;
  if (std::abs(effective_e2) < 0.015) {
    effective_e2 = 0.0;
  }
  double effective_e3 = error.e3;
  if (std::abs(effective_e3) < 0.025) {
    effective_e3 = 0.0;
  }

  // omega_d = omega_r + k2 * v_r * e2 + k3 * v_r * sin(e3)
  cmd.omega = omega_r + (gains.k2 * v_r * effective_e2) + (gains.k3 * v_r * std::sin(effective_e3));

  return cmd;
}

double KinematicLaw::computeTerminalRotation(
  double current_yaw,
  double goal_yaw,
  const TerminalParams & params,
  ControllerState & out_state)
{
  double e_theta = normalizeAngle(goal_yaw - current_yaw);

  if (std::abs(e_theta) <= params.yaw_goal_tolerance) {
    out_state = ControllerState::GOAL_REACHED;
    return 0.0;
  }

  out_state = ControllerState::TERMINAL_ROTATE;
  double omega_cmd = params.k_theta * e_theta;

  // Apply maximum saturation
  if (params.max_angular_vel > 0.0) {
    omega_cmd = std::clamp(omega_cmd, -params.max_angular_vel, params.max_angular_vel);
  }

  // Enforce minimum executable rotation speed if configured and outside tolerance
  if (params.min_angular_vel > 0.0 && std::abs(omega_cmd) < params.min_angular_vel) {
    omega_cmd = (omega_cmd >= 0.0) ? params.min_angular_vel : -params.min_angular_vel;
  }

  return omega_cmd;
}

WheelRPM KinematicLaw::computeInverseKinematics(
  const Velocity2D & cmd,
  const RobotParameters & params)
{
  WheelRPM rpm;
  if (params.r <= 0.0) {
    return rpm;
  }

  // Paper Eq. (15 - 16)
  // omega_R = (v_d + b * omega_d) / r
  // omega_L = (v_d - b * omega_d) / r
  double omega_r_radps = (cmd.v + params.b * cmd.omega) / params.r;
  double omega_l_radps = (cmd.v - params.b * cmd.omega) / params.r;

  // Convert rad/s to RPM: RPM = omega * 60 / (2*pi)
  rpm.right = (omega_r_radps * 60.0) / (2.0 * M_PI);
  rpm.left = (omega_l_radps * 60.0) / (2.0 * M_PI);

  return rpm;
}

Velocity2D KinematicLaw::applySaturation(
  const Velocity2D & cmd,
  double max_v,
  double max_omega)
{
  Velocity2D sat_cmd = cmd;

  if (max_v > 0.0) {
    sat_cmd.v = std::clamp(sat_cmd.v, -max_v, max_v);
  }

  if (max_omega > 0.0) {
    sat_cmd.omega = std::clamp(sat_cmd.omega, -max_omega, max_omega);
  }

  return sat_cmd;
}

bool KinematicLaw::isTrajectorySafe(
  double x0,
  double y0,
  double theta0,
  double v,
  double omega,
  const nav2_costmap_2d::Footprint & footprint,
  nav2_costmap_2d::Costmap2D * costmap,
  const CollisionCheckParams & params)
{
  if (!costmap) {
    return true;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);

  int num_steps = static_cast<int>(std::ceil(params.horizon / params.dt));
  if (num_steps < 1) num_steps = 1;

  double x = x0;
  double y = y0;
  double theta = theta0;

  for (int i = 1; i <= num_steps; ++i) {
    x += v * std::cos(theta) * params.dt;
    y += v * std::sin(theta) * params.dt;
    theta = normalizeAngle(theta + omega * params.dt);

    double cost = checker.footprintCostAtPose(x, y, theta, footprint);

    // Negative cost indicates out of bounds
    if (cost < 0.0) {
      return false;
    }

    // Cost 255 is NO_INFORMATION (Unknown cell)
    if (cost == 255.0) {
      if (params.treat_unknown_as_unsafe) {
        return false;
      }
      // If treat_unknown_as_unsafe is false, unknown space is allowed
      continue;
    }

    // Cost 254 (LETHAL_OBSTACLE) or 253 (INSCRIBED_INFLATED_OBSTACLE)
    if (cost >= 253.0) {
      return false;
    }
  }

  return true;
}

bool KinematicLaw::selectBestEscapeDirection(
  const Pose2D & robot_pose,
  const Pose2D & goal_pose,
  const nav2_costmap_2d::Footprint & footprint,
  nav2_costmap_2d::Costmap2D * costmap,
  const EscapeRecoveryParams & params,
  const CollisionCheckParams & collision_params,
  EscapeCandidate & out_best_candidate,
  std::vector<EscapeCandidate> & out_all_candidates)
{
  out_all_candidates.clear();
  if (!costmap) {
    return false;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(costmap);

  // Define 8 Discrete Candidate Motions (A..H)
  std::vector<EscapeCandidate> candidates = {
    // A. Forward-Left Arc
    {EscapeMotionType::FORWARD_LEFT_ARC, "forward_left_arc", params.linear_speed, params.arc_angular_speed, params.escape_distance, 0.0, true, 0.0, 1.0, -1e9},
    // B. Forward-Right Arc
    {EscapeMotionType::FORWARD_RIGHT_ARC, "forward_right_arc", params.linear_speed, -params.arc_angular_speed, params.escape_distance, 0.0, true, 0.0, 1.0, -1e9},
    // C. Reverse-Left Arc
    {EscapeMotionType::REVERSE_LEFT_ARC, "reverse_left_arc", -params.linear_speed, -params.arc_angular_speed, params.escape_distance, 0.0, true, 0.0, 1.0, -1e9},
    // D. Reverse-Right Arc
    {EscapeMotionType::REVERSE_RIGHT_ARC, "reverse_right_arc", -params.linear_speed, params.arc_angular_speed, params.escape_distance, 0.0, true, 0.0, 1.0, -1e9},
    // E. Straight Reverse
    {EscapeMotionType::STRAIGHT_REVERSE, "straight_reverse", -params.linear_speed, 0.0, params.escape_distance, 0.0, true, 0.0, 1.0, -1e9},
    // F. Rotate-Left in Place
    {EscapeMotionType::ROTATE_LEFT, "rotate_left", 0.0, params.angular_speed, 0.0, params.rotation_angle, true, 0.0, 1.0, -1e9},
    // G. Rotate-Right in Place
    {EscapeMotionType::ROTATE_RIGHT, "rotate_right", 0.0, -params.angular_speed, 0.0, params.rotation_angle, true, 0.0, 1.0, -1e9},
    // H. Straight Forward
    {EscapeMotionType::STRAIGHT_FORWARD, "straight_forward", params.linear_speed, 0.0, params.escape_distance, 0.0, true, 0.0, 1.0, -1e9}
  };

  bool found_safe = false;
  double best_score = -1e9;
  size_t best_idx = 0;
  const double sim_dt = 0.02; // 20ms fine simulation step

  for (size_t i = 0; i < candidates.size(); ++i) {
    EscapeCandidate & cand = candidates[i];
    cand.is_safe = true;
    cand.max_cost = 0.0;

    double duration = (std::abs(cand.v) > 1e-4) ?
      (cand.target_dist / std::abs(cand.v)) :
      (cand.target_angle / std::abs(cand.omega));

    int num_steps = static_cast<int>(std::ceil(duration / sim_dt));
    if (num_steps < 1) num_steps = 1;

    double sim_x = robot_pose.x;
    double sim_y = robot_pose.y;
    double sim_th = robot_pose.theta;

    // Full swept footprint check over the entire trajectory
    for (int s = 1; s <= num_steps; ++s) {
      sim_x += cand.v * std::cos(sim_th) * sim_dt;
      sim_y += cand.v * std::sin(sim_th) * sim_dt;
      sim_th = normalizeAngle(sim_th + cand.omega * sim_dt);

      double cost = checker.footprintCostAtPose(sim_x, sim_y, sim_th, footprint);

      if (cost < 0.0 || cost >= 253.0 || (cost == 255.0 && collision_params.treat_unknown_as_unsafe)) {
        cand.is_safe = false;
        break;
      }
      if (cost > cand.max_cost && cost != 255.0) {
        cand.max_cost = cost;
      }
    }

    if (cand.is_safe) {
      cand.min_clearance = (254.0 - cand.max_cost) / 254.0;

      // Calculate goal alignment from end pose of maneuver
      double dx_g = goal_pose.x - sim_x;
      double dy_g = goal_pose.y - sim_y;
      double goal_heading = std::atan2(dy_g, dx_g);
      double heading_err = normalizeAngle(goal_heading - sim_th);

      double goal_alignment = 0.0;
      if (cand.v > 0.0) {
        goal_alignment = std::cos(heading_err);
      } else if (cand.v < 0.0) {
        goal_alignment = std::cos(M_PI - heading_err) - 0.2;
      } else {
        goal_alignment = std::cos(heading_err);
      }

      // Score: Safety dominates (clearance), then goal alignment, small penalty for unnecessary reverse/rotation
      double reverse_penalty = (cand.v < 0.0) ? -0.25 : 0.0;
      double turn_penalty = (std::abs(cand.omega) > 0.0) ? -0.15 : 0.0;

      cand.score = 4.0 * cand.min_clearance + 1.5 * goal_alignment + reverse_penalty + turn_penalty;

      if (!found_safe || cand.score > best_score) {
        found_safe = true;
        best_score = cand.score;
        best_idx = i;
      }
    }

    out_all_candidates.push_back(cand);
  }

  if (found_safe) {
    out_best_candidate = out_all_candidates[best_idx];
  }

  return found_safe;
}

}  // namespace plan_c_kinematic_controller
