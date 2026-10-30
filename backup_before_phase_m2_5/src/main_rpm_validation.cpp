#include <Arduino.h>
#include "motor.h"
#include "motor_rpm_validation.h"

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

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Khởi tạo Motor Driver
  motors.begin();

  // Khởi tạo ngắt ISR đếm xung
  setupValidationInterrupts();

  Serial.println("\n===== BẮT ĐẦU PROGRAM TEST PHASE 2B.1: REPEATED MOTOR RPM VALIDATION (5 RUNS/PWM) =====");
}

void loop() {
  // Chạy máy trạng thái kiểm chứng lặp lại RPM Phase 2B.1
  runMotorRPMValidation(motors);
}
