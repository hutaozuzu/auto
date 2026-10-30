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

  // omega_d = omega_r + k2 * v_r * e2 + k3 * v_r * sin(e3)
  cmd.omega = omega_r + (gains.k2 * v_r * error.e2) + (gains.k3 * v_r * std::sin(error.e3));

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

    // Negative cost or cost >= 253.0 indicates obstacle collision / inscribed inflation / out of bounds
    if (cost < 0.0 || cost >= 253.0) {
      return false;
    }

    if (params.treat_unknown_as_unsafe && cost == 255.0) {
      return false;
    }
  }

  return true;
}

}  // namespace plan_c_kinematic_controller
