#include <gtest/gtest.h>
#include <cmath>
#include "plan_c_kinematic_controller/kinematic_law.hpp"

using namespace plan_c_kinematic_controller;

// TEST A — ZERO ERROR
TEST(KinematicLawTest, TestA_ZeroError)
{
  Pose2D actual{1.0, 2.0, 0.5};
  Pose2D reference{1.0, 2.0, 0.5};
  double v_r = 0.15;
  double omega_r = 0.10;
  ControllerGains gains{1.5, 2.0, 1.0};

  TrackingError err = KinematicLaw::computeTrackingError(actual, reference);
  EXPECT_NEAR(err.e1, 0.0, 1e-6);
  EXPECT_NEAR(err.e2, 0.0, 1e-6);
  EXPECT_NEAR(err.e3, 0.0, 1e-6);

  Velocity2D cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains);
  EXPECT_NEAR(cmd.v, v_r, 1e-6);
  EXPECT_NEAR(cmd.omega, omega_r, 1e-6);
}

// TEST B — LONGITUDINAL ERROR
TEST(KinematicLawTest, TestB_LongitudinalError)
{
  Pose2D actual{0.0, 0.0, 0.0};
  Pose2D reference{0.20, 0.0, 0.0};  // Reference directly 20cm in front
  double v_r = 0.15;
  double omega_r = 0.0;
  ControllerGains gains{1.5, 2.0, 1.0};

  TrackingError err = KinematicLaw::computeTrackingError(actual, reference);
  EXPECT_GT(err.e1, 0.19);
  EXPECT_NEAR(err.e2, 0.0, 1e-6);
  EXPECT_NEAR(err.e3, 0.0, 1e-6);

  Velocity2D cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains);
  // v_d = v_r * cos(0) + k1 * e1 > v_r
  EXPECT_GT(cmd.v, v_r);
  EXPECT_NEAR(cmd.omega, 0.0, 1e-6);
}

// TEST C — LATERAL ERROR
TEST(KinematicLawTest, TestC_LateralError)
{
  Pose2D actual{0.0, 0.0, 0.0};
  Pose2D reference{0.0, 0.10, 0.0};  // Reference is 10cm to robot's left
  double v_r = 0.15;
  double omega_r = 0.0;
  ControllerGains gains{1.5, 2.0, 1.0};

  TrackingError err = KinematicLaw::computeTrackingError(actual, reference);
  EXPECT_NEAR(err.e1, 0.0, 1e-6);
  EXPECT_GT(err.e2, 0.09);  // e2 > 0
  EXPECT_NEAR(err.e3, 0.0, 1e-6);

  Velocity2D cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains);
  // omega_d = omega_r + k2 * v_r * e2 > 0 (turns left toward reference)
  EXPECT_GT(cmd.omega, 0.0);
}

// TEST D — HEADING ERROR
TEST(KinematicLawTest, TestD_HeadingError)
{
  Pose2D actual{0.0, 0.0, 0.0};
  Pose2D reference{0.0, 0.0, 0.20};  // Reference is rotated CCW by 0.20 rad
  double v_r = 0.15;
  double omega_r = 0.0;
  ControllerGains gains{1.5, 2.0, 1.0};

  TrackingError err = KinematicLaw::computeTrackingError(actual, reference);
  EXPECT_NEAR(err.e1, 0.0, 1e-6);
  EXPECT_NEAR(err.e2, 0.0, 1e-6);
  EXPECT_GT(err.e3, 0.19);  // e3 > 0

  Velocity2D cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains);
  // sin(e3) contribution gives positive omega correction (CCW)
  EXPECT_GT(cmd.omega, 0.0);
}

// TEST E — ANGLE WRAP
TEST(KinematicLawTest, TestE_AngleWrap)
{
  double theta_actual = 179.0 * M_PI / 180.0;     // +179 deg
  double theta_reference = -179.0 * M_PI / 180.0;  // -179 deg
  Pose2D actual{0.0, 0.0, theta_actual};
  Pose2D reference{0.0, 0.0, theta_reference};

  TrackingError err = KinematicLaw::computeTrackingError(actual, reference);
  // Expected shortest angular error: +2.0 deg = +0.0349 rad (NOT -358 deg)
  double expected_err = 2.0 * M_PI / 180.0;
  EXPECT_NEAR(err.e3, expected_err, 1e-4);
  EXPECT_LT(std::abs(err.e3), M_PI);
}

// TEST F — INVERSE KINEMATICS
TEST(KinematicLawTest, TestF_InverseKinematics)
{
  Velocity2D cmd{0.10, 0.0};
  RobotParameters params{0.0325, 0.0550};  // r = 0.0325m, b = 0.055m

  WheelRPM rpm = KinematicLaw::computeInverseKinematics(cmd, params);
  // Straight motion: left RPM == right RPM
  EXPECT_NEAR(rpm.left, rpm.right, 1e-6);
  double expected_rpm = (0.10 / (2.0 * M_PI * 0.0325)) * 60.0;  // ~29.38 RPM
  EXPECT_NEAR(rpm.left, expected_rpm, 1e-2);
}

// TEST G — DIFFERENTIAL TURN
TEST(KinematicLawTest, TestG_DifferentialTurn)
{
  Velocity2D cmd{0.10, 0.30};  // Moving forward with CCW turn
  RobotParameters params{0.0325, 0.0550};

  WheelRPM rpm = KinematicLaw::computeInverseKinematics(cmd, params);
  // For positive omega (CCW turn), right wheel must rotate faster than left wheel
  EXPECT_GT(rpm.right, rpm.left);
  EXPECT_GT(rpm.right, 30.0);
  EXPECT_LT(rpm.left, 28.0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
