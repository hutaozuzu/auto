#ifndef PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_LAW_HPP_
#define PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_LAW_HPP_

#include <cmath>

namespace plan_c_kinematic_controller
{

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

/**
 * @brief Pure mathematical implementation of the whole-robot nonholonomic kinematic tracking control law.
 * Based on Paper: "Designing a robust adaptive controller for a differentially driven nonholonomic wheeled mobile robot tracking trajectory"
 * (Kinematic Section, Equations 13 - 16).
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
};

}  // namespace plan_c_kinematic_controller

#endif  // PLAN_C_KINEMATIC_CONTROLLER__KINEMATIC_LAW_HPP_
