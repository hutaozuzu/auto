#include "plan_c_kinematic_controller/kinematic_law.hpp"
#include <algorithm>

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

}  // namespace plan_c_kinematic_controller
