#include <Arduino.h>
#include "motor.h"
#include "speed_controller.h"
#include "speed_control_test.h"
#include "odometry_calibration.h"

// Động cơ (Khởi tạo chính xác sơ đồ chân GPIO phần cứng)
// PWMA=4, AIN1=18, AIN2=19, PWMB=5, BIN1=25, BIN2=26, STBY=23
static MotorDriver motors(4, 18, 19, 5, 25, 26, 23);

// Bộ điều khiển vòng kín PI + Feedforward cho 2 bánh
static SpeedController leftController(true,  INITIAL_KP_LEFT,  INITIAL_KI_LEFT);
static SpeedController rightController(false, INITIAL_KP_RIGHT, INITIAL_KI_RIGHT);

void setup() {
  Serial.begin(115200);
  delay(1000);

  motors.begin();
  motors.stop();

  setupOdometryInterrupts();
}

void loop() {
  runOdometryCalibration(motors, leftController, rightController);
}
