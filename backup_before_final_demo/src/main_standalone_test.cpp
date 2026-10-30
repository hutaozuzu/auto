#include <Arduino.h>
#include "motor.h"
#include "encoder_test.h"

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

// Số xung thực tế đo được chính xác cho 1 vòng quay bánh xe từ nghiệm thu phần cứng thực tế
#define LEFT_PULSES_PER_REV   38
#define RIGHT_PULSES_PER_REV  38

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Khởi tạo Motor Driver
  motors.begin();

  // Khởi tạo ngắt ISR cho Encoder FC-03
  setupEncoderInterrupts();

  Serial.println("\n===== CHƯƠNG TRÌNH STANDALONE TEST: MOTOR TB6612FNG + FC-03 ENCODER =====");
}

void loop() {
  // Chạy kịch bản test Standalone
  runStandaloneEncoderMotorTest(motors, LEFT_PULSES_PER_REV, RIGHT_PULSES_PER_REV);
}
