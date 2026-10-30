#ifndef MOTOR_SPEED_TEST_H
#define MOTOR_SPEED_TEST_H

#include <Arduino.h>
#include "motor.h"

// ==========================================
// CẤU HÌNH GPIO ENCODER FC-03 & THÔNG SỐ ĐO
// ==========================================
#define ENCODER_LEFT_PIN       34   // GPI 34: Input Only
#define ENCODER_RIGHT_PIN      35   // GPI 35: Input Only

#define PULSES_PER_REV         38   // Đã nghiệm thu thực tế 38 xung/vòng
#define MEASURE_WINDOW_MS      1000 // Cửa sổ đo RPM 1000ms (1 giây)
#define WARMUP_TIME_MS         2000 // Thời gian warm-up 2 giây cho motor ổn định tốc độ
#define REST_TIME_MS           1000 // Thời gian nghỉ 1 giây giữa các mức PWM

#define STALL_CHECK_TIMEOUT_MS 1500 // Cảnh báo/Tắt ngắt nếu không có xung trong 1.5s lúc đang chạy PWM

// Danh sách 7 mức PWM thử nghiệm
const uint8_t PWM_TEST_POINTS[] = {65, 90, 120, 150, 180, 210, 255};
const uint8_t NUM_PWM_STEPS = sizeof(PWM_TEST_POINTS) / sizeof(PWM_TEST_POINTS[0]);

// Các trạng thái máy trạng thái đo RPM
enum SpeedTestState {
  SPEED_TEST_INIT,
  SPEED_TEST_LEFT_WARMUP,
  SPEED_TEST_LEFT_MEASURE,
  SPEED_TEST_LEFT_REST,
  SPEED_TEST_RIGHT_WARMUP,
  SPEED_TEST_RIGHT_MEASURE,
  SPEED_TEST_RIGHT_REST,
  SPEED_TEST_DONE
};

/**
 * @brief Khởi tạo ngắt ISR đếm xung cho Encoder FC-03
 */
void setupSpeedTestInterrupts();

/**
 * @brief Máy trạng thái đo đặc tính tốc độ PWM -> RPM cho Motor Trái & Motor Phải
 * 
 * @param motors Tham chiếu đến đối tượng MotorDriver
 */
void runMotorSpeedCharacterization(MotorDriver& motors);

#endif // MOTOR_SPEED_TEST_H
