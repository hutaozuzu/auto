#ifndef ENCODER_TEST_H
#define ENCODER_TEST_H

#include <Arduino.h>
#include "motor.h"

// ==========================================
// CẤU HÌNH GPIO ENCODER FC-03
// ==========================================
#define ENCODER_LEFT_PIN   34  // GPI 34: Input Only (Dùng cho Encoder Trái)
#define ENCODER_RIGHT_PIN  35  // GPI 35: Input Only (Dùng cho Encoder Phải)

// Tốc độ PWM thử nghiệm mặc định (Giảm xuống 65 để quay chậm, dễ quan sát)
#define DEFAULT_TEST_PWM   65

// Timeout an toàn (Tính bằng miligiây)
#define MOTOR_SAFETY_TIMEOUT_MS  5000  // Dừng motor nếu quá 5 giây mà chưa đạt target pulse
#define ENCODER_STALL_TIMEOUT_MS  1000  // Cảnh báo kẹt bánh nếu sau 1s motor quay mà 0 có xung nào

// Các chế độ hoạt động trong chương trình Test Standalone (Điều khiển thủ công bằng phím 'n')
enum TestMode {
  MODE_READY_LEFT,       // Chờ bấm 'n' để cho Motor Trái quay
  MODE_RUNNING_LEFT,     // Motor Trái đang quay, chờ tự dừng HOẶC bấm 'n' để dừng
  MODE_READY_RIGHT,      // Chờ bấm 'n' để cho Motor Phải quay
  MODE_RUNNING_RIGHT,    // Motor Phải đang quay, chờ tự dừng HOẶC bấm 'n' để dừng
  MODE_COMPLETED         // Hoàn thành bài test
};

/**
 * @brief Khởi tạo ngắt ISR cho Encoder FC-03
 */
void setupEncoderInterrupts();

/**
 * @brief Chạy máy trạng thái Standalone Test Động cơ + Encoder
 * 
 * @param motors Tham chiếu đến đối tượng MotorDriver
 * @param target_left_pulses Số xung cài đặt cho 1 vòng Motor Trái (Mặc định 20)
 * @param target_right_pulses Số xung cài đặt cho 1 vòng Motor Phải (Mặc định 20)
 */
void runStandaloneEncoderMotorTest(MotorDriver& motors, uint32_t target_left_pulses = 20, uint32_t target_right_pulses = 20);

#endif // ENCODER_TEST_H
