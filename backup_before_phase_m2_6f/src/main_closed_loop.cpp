#include <Arduino.h>
#include "motor.h"
#include "speed_controller.h"
#include "speed_control_test.h"

// ==========================================
// CẤU HÌNH CHÂN DÒNG ĐỘNG CƠ TB6612FNG (SAFE GPIOS)
// ==========================================
#define MOTOR_PWMA  4   // PWM Động cơ Trái (Motor A)
#define MOTOR_AIN1  18  // IN1 Động cơ Trái
#define MOTOR_AIN2  19  // IN2 Động cơ Trái

#define MOTOR_PWMB  5   // PWM Động cơ Phải (Motor B)
#define MOTOR_BIN1  25  // IN1 Động cơ Phải
#define MOTOR_BIN2  26  // IN2 Động cơ Phải

#define MOTOR_STBY  23  // Chân Standby kích hoạt TB6612FNG

MotorDriver motors(MOTOR_PWMA, MOTOR_AIN1, MOTOR_AIN2,
                    MOTOR_PWMB, MOTOR_BIN1, MOTOR_BIN2,
                    MOTOR_STBY);

// Khởi tạo SpeedController cho Motor Trái (is_left = true) và Phải (is_left = false)
SpeedController leftController(true, INITIAL_KP_LEFT, INITIAL_KI_LEFT, 0, 210);
SpeedController rightController(false, INITIAL_KP_RIGHT, INITIAL_KI_RIGHT, 0, 210);

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Khởi tạo MotorDriver
  motors.begin();

  // Khởi tạo ngắt đếm xung
  setupSpeedControlInterrupts();

  Serial.println("\n===== CHƯƠNG TRÌNH STANDALONE TEST PHASE 2C: CLOSED-LOOP WHEEL SPEED CONTROL =====");
}

void loop() {
  // Chạy kịch bản điều khiển vòng kín PI + Feedforward Phase 2C
  runClosedLoopSpeedTest(motors, leftController, rightController);
}
