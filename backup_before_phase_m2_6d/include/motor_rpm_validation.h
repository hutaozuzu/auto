#ifndef MOTOR_RPM_VALIDATION_H
#define MOTOR_RPM_VALIDATION_H

#include <Arduino.h>
#include "motor.h"

// ==========================================
// CẤU HÌNH GPIO ENCODER FC-03 & THÔNG SỐ ĐO
// ==========================================
#define ENCODER_LEFT_PIN       34   // GPI 34: Input Only (Left Encoder)
#define ENCODER_RIGHT_PIN      35   // GPI 35: Input Only (Right Encoder)

#define PULSES_PER_REV         38   // Đã nghiệm thu thực tế 38 xung/vòng
#define MEASUREMENT_WINDOW_MS  2000 // Cửa sổ thời gian đo 2000ms (2 giây) cho độ phân giải RPM mịn hơn
#define WARMUP_TIME_MS         2000 // Thời gian warm-up 2 giây cho motor đạt vận tốc ổn định
#define REST_TIME_MS           1000 // Thời gian nghỉ 1 giây giữa các lần đo
#define RUNS_PER_PWM           5    // Đo lặp lại đúng 5 lần độc lập cho mỗi mức PWM

#define STALL_CHECK_TIMEOUT_MS 1500 // Cảnh báo kẹt bánh nếu sau 1.5s không nhận được xung nào lúc motor đang bật PWM

// Danh sách các mức PWM thử nghiệm (7 mức)
const uint8_t VALIDATION_PWM_STEPS[] = {65, 90, 120, 150, 180, 210, 255};
const uint8_t NUM_VALIDATION_STEPS = sizeof(VALIDATION_PWM_STEPS) / sizeof(VALIDATION_PWM_STEPS[0]);

// Các trạng thái máy trạng thái kiểm chứng RPM lặp lại Phase 2B.1
enum ValidationState {
  VAL_INIT,
  VAL_LEFT_WARMUP,
  VAL_LEFT_MEASURE,
  VAL_LEFT_REST,
  VAL_RIGHT_WARMUP,
  VAL_RIGHT_MEASURE,
  VAL_RIGHT_REST,
  VAL_DONE
};

/**
 * @brief Khởi tạo ngắt ISR đếm xung cho Encoder FC-03
 */
void setupValidationInterrupts();

/**
 * @brief Máy trạng thái đo lặp lại 5 lần đặc tính PWM -> RPM cho Motor Trái & Motor Phải
 * 
 * @param motors Tham chiếu đến đối tượng MotorDriver
 */
void runMotorRPMValidation(MotorDriver& motors);

#endif // MOTOR_RPM_VALIDATION_H
