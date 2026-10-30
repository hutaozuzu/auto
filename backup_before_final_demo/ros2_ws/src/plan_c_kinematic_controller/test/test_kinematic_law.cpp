#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "plan_c_kinematic_controller/kinematic_law.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace plan_c_kinematic_controller;

// Helper to create rectangular footprint matching robot geometry
nav2_costmap_2d::Footprint makeRobotFootprint()
{
  nav2_costmap_2d::Footprint fp;
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = -0.08; p1.y = -0.08; p1.z = 0.0;
  p2.x =  0.08; p2.y = -0.08; p2.z = 0.0;
  p3.x =  0.08; p3.y =  0.08; p3.z = 0.0;
  p4.x = -0.08; p4.y =  0.08; p4.z = 0.0;
  fp.push_back(p1);
  fp.push_back(p2);
  fp.push_back(p3);
  fp.push_back(p4);
  return fp;
}

// =========================================================================
// TESTS A - G: ORIGINAL PHASE K1 BASELINE TESTS
// =========================================================================

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
  EXPECT_GT(err.e2, 0.09);
  EXPECT_NEAR(err.e3, 0.0, 1e-6);

  Velocity2D cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains);
  EXPECT_GT(cmd.omega, 0.0);
}

// TEST D — HEADING ERROR
TEST(KinematicLawTest, TestD_HeadingError)
{
  Pose2D actual{0.0, 0.0, 0.0};
  Pose2D reference{0.0, 0.0, 0.20};  // Reference rotated CCW by 0.20 rad
  double v_r = 0.15;
  double omega_r = 0.0;
  ControllerGains gains{1.5, 2.0, 1.0};

  TrackingError err = KinematicLaw::computeTrackingError(actual, reference);
  EXPECT_NEAR(err.e1, 0.0, 1e-6);
  EXPECT_NEAR(err.e2, 0.0, 1e-6);
  EXPECT_GT(err.e3, 0.19);

  Velocity2D cmd = KinematicLaw::computeControlLaw(err, v_r, omega_r, gains);
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
  EXPECT_NEAR(rpm.left, rpm.right, 1e-6);
  double expected_rpm = (0.10 / (2.0 * M_PI * 0.0325)) * 60.0;
  EXPECT_NEAR(rpm.left, expected_rpm, 1e-2);
}

// TEST G — DIFFERENTIAL TURN
TEST(KinematicLawTest, TestG_DifferentialTurn)
{
  Velocity2D cmd{0.10, 0.30};
  RobotParameters params{0.0325, 0.0550};

  WheelRPM rpm = KinematicLaw::computeInverseKinematics(cmd, params);
  EXPECT_GT(rpm.right, rpm.left);
  EXPECT_GT(rpm.right, 30.0);
  EXPECT_LT(rpm.left, 28.0);
}

// =========================================================================
// TESTS H - K: PHASE K1.1 TERMINAL GOAL ORIENTATION TESTS
// =========================================================================

// TEST H — XY REACHED / YAW NOT REACHED (+30 deg)
TEST(KinematicLawTest, TestH_TerminalRotatePositive)
{
  double current_yaw = 0.0;
  double goal_yaw = 30.0 * M_PI / 180.0;  // +30 deg
  TerminalParams params{1.5, 0.80, 0.0, 0.15, 0.15};
  ControllerState state = ControllerState::TRACK_PATH;

  double omega_cmd = KinematicLaw::computeTerminalRotation(current_yaw, goal_yaw, params, state);
  EXPECT_EQ(state, ControllerState::TERMINAL_ROTATE);
  EXPECT_GT(omega_cmd, 0.0);
  EXPECT_LE(omega_cmd, params.max_angular_vel);
}

// TEST I — NEGATIVE YAW ERROR (-30 deg)
TEST(KinematicLawTest, TestI_TerminalRotateNegative)
{
  double current_yaw = 0.0;
  double goal_yaw = -30.0 * M_PI / 180.0;  // -30 deg
  TerminalParams params{1.5, 0.80, 0.0, 0.15, 0.15};
  ControllerState state = ControllerState::TRACK_PATH;

  double omega_cmd = KinematicLaw::computeTerminalRotation(current_yaw, goal_yaw, params, state);
  EXPECT_EQ(state, ControllerState::TERMINAL_ROTATE);
  EXPECT_LT(omega_cmd, 0.0);
  EXPECT_GE(omega_cmd, -params.max_angular_vel);
}

// TEST J — YAW WITHIN TOLERANCE
TEST(KinematicLawTest, TestJ_TerminalGoalReached)
{
  double current_yaw = 0.0;
  double goal_yaw = 0.05;  // 2.86 deg (< 0.15 rad tolerance)
  TerminalParams params{1.5, 0.80, 0.0, 0.15, 0.15};
  ControllerState state = ControllerState::TERMINAL_ROTATE;

  double omega_cmd = KinematicLaw::computeTerminalRotation(current_yaw, goal_yaw, params, state);
  EXPECT_EQ(state, ControllerState::GOAL_REACHED);
  EXPECT_NEAR(omega_cmd, 0.0, 1e-6);
}

// TEST K — ANGLE WRAP TERMINAL
TEST(KinematicLawTest, TestK_TerminalAngleWrap)
{
  double current_yaw = 170.0 * M_PI / 180.0;   // +170 deg
  double goal_yaw = -170.0 * M_PI / 180.0;     // -170 deg
  TerminalParams params{1.5, 0.80, 0.0, 0.15, 0.15};
  ControllerState state = ControllerState::TRACK_PATH;

  double omega_cmd = KinematicLaw::computeTerminalRotation(current_yaw, goal_yaw, params, state);
  // Shortest angular turn is +20.0 deg (positive omega), NOT -340 deg
  EXPECT_EQ(state, ControllerState::TERMINAL_ROTATE);
  EXPECT_GT(omega_cmd, 0.0);
  EXPECT_LT(std::abs(omega_cmd), 0.80);
}

// =========================================================================
// TESTS L - P: PHASE K1.1 FORWARD TRAJECTORY COLLISION TESTS
// =========================================================================

// TEST L — EMPTY FREE COSTMAP
TEST(KinematicLawTest, TestL_EmptyFreeCostmap)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, false};

  bool safe = KinematicLaw::isTrajectorySafe(1.0, 1.0, 0.0, 0.15, 0.0, footprint, &costmap, params);
  EXPECT_TRUE(safe);
}

// TEST M — OBSTACLE DIRECTLY AHEAD
TEST(KinematicLawTest, TestM_ObstacleDirectlyAhead)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, false};

  // Place lethal obstacle at (1.15m, 1.00m) - directly on forward path from (1.00m, 1.00m)
  unsigned int mx, my;
  costmap.worldToMap(1.15, 1.00, mx, my);
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  bool safe = KinematicLaw::isTrajectorySafe(1.0, 1.0, 0.0, 0.15, 0.0, footprint, &costmap, params);
  EXPECT_FALSE(safe);  // Collision detected
}

// TEST N — OBSTACLE BESIDE PATH
TEST(KinematicLawTest, TestN_ObstacleBesidePath)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, false};

  // Place lethal obstacle at (1.15m, 1.50m) - 50cm to the left, well outside footprint [-0.08, 0.08]
  unsigned int mx, my;
  costmap.worldToMap(1.15, 1.50, mx, my);
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  bool safe = KinematicLaw::isTrajectorySafe(1.0, 1.0, 0.0, 0.15, 0.0, footprint, &costmap, params);
  EXPECT_TRUE(safe);  // Safe
}

// TEST O — ROTATION SWEEP
TEST(KinematicLawTest, TestO_RotationSweepCollision)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, false};

  // Robot at (1.00m, 1.00m, 0.0). Footprint extends to corners (+-0.08m, +-0.08m) = radius ~0.113m.
  // Place obstacle at (1.08m, 1.08m)
  unsigned int mx, my;
  costmap.worldToMap(1.08, 1.08, mx, my);
  costmap.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);

  // When rotating in place, corner sweeps through (1.08, 1.08)
  bool safe = KinematicLaw::isTrajectorySafe(1.0, 1.0, 0.0, 0.0, 1.0, footprint, &costmap, params);
  EXPECT_FALSE(safe);  // Rotational collision detected
}

// TEST P — OUTSIDE COSTMAP BOUNDS
TEST(KinematicLawTest, TestP_OutsideCostmapBounds)
{
  nav2_costmap_2d::Costmap2D costmap(20, 20, 0.05, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE); // 1.0m x 1.0m
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, false};

  // Robot at edge (0.90m, 0.90m) driving forward at 0.50 m/s -> leaves costmap bounds
  bool safe = KinematicLaw::isTrajectorySafe(0.90, 0.90, 0.0, 0.50, 0.0, footprint, &costmap, params);
  EXPECT_FALSE(safe);  // Unsafe due to out-of-bounds
}
// TEST Q — UNKNOWN SPACE ALLOWED (treat_unknown_as_unsafe = false)
TEST(KinematicLawTest, TestQ_UnknownSpaceAllowed)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, false}; // treat_unknown_as_unsafe = false

  bool safe = KinematicLaw::isTrajectorySafe(1.0, 1.0, 0.0, 0.15, 0.0, footprint, &costmap, params);
  EXPECT_TRUE(safe);  // Unknown space is allowed when policy is false
}

// TEST R — UNKNOWN SPACE REJECTED (treat_unknown_as_unsafe = true)
TEST(KinematicLawTest, TestR_UnknownSpaceUnsafe)
{
  nav2_costmap_2d::Costmap2D costmap(100, 100, 0.05, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
  auto footprint = makeRobotFootprint();
  CollisionCheckParams params{0.8, 0.05, true}; // treat_unknown_as_unsafe = true

  bool safe = KinematicLaw::isTrajectorySafe(1.0, 1.0, 0.0, 0.15, 0.0, footprint, &costmap, params);
  EXPECT_FALSE(safe);  // Unknown space rejected when policy is true
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
