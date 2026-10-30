#ifndef ODOMETRY_CALIBRATION_H
#define ODOMETRY_CALIBRATION_H

#include <Arduino.h>
#include "motor.h"
#include "speed_controller.h"

// ==========================================
// THÔNG SỐ CƠ KHÍ & ENCODER (SI UNITS: METERS, SECONDS)
// ==========================================
#define ENCODER_LEFT_PIN        34    // GPI 34
#define ENCODER_RIGHT_PIN       35    // GPI 35

#define PULSES_PER_REV          38    // Đã xác nhận thực nghiệm 38 xung / vòng
#define CONTROL_PERIOD_MS       200   // Chu kỳ điều khiển vòng kín cố định 200 ms (5 Hz)

// Cấu hình kích thước cơ khí thực tế (Có thể chỉnh sửa dựa trên đo đạc thực tế)
#define DEFAULT_WHEEL_DIAMETER_M   0.065f  // Đường kính bánh xe: 0.065m (65mm)
#define DEFAULT_WHEEL_SEPARATION_M 0.110f  // Khoảng cách giữa 2 tâm bánh xe đo thực tế: 0.110m (110mm)

// Target RPM thử nghiệm bám ổn định
#define CALIB_TARGET_RPM        100.0f

#define COUNTDOWN_WAIT_MS            5000   // Chờ 5 giây sau khi bật nguồn/reset để người dùng đặt robot
#define EXACT_RUN_DURATION_MS        3000   // Thời gian phát lệnh Target RPM chính xác 3.000 ms (3.0s)
#define ODOM_UPDATE_PERIOD_MS        100    // Chu kỳ update Pose integration cố định: 100 ms (10 Hz)
#define COASTING_TIMEOUT_MS          3000   // Thời gian chờ trôi quán tính tối đa (3.0s)
#define STOP_CONFIRM_SAMPLES         3      // 3 chu kỳ liên tiếp (600ms) không có delta xung mới = BÁNH DỪNG HẲN

// CẤU HÌNH CÁC CHẾ ĐỘ TEST PHASE 3B
enum Phase3BTestMode {
  TEST_MODE_STRAIGHT,      // Test A: Chạy thẳng 100 RPM
  TEST_MODE_ROTATE_LEFT,   // Test B: Quay trái tại chỗ (Motor Trái lùi 100 RPM, Motor Phải tiến 100 RPM)
  TEST_MODE_ROTATE_RIGHT   // Test C: Quay phải tại chỗ (Motor Trái tiến 100 RPM, Motor Phải lùi 100 RPM)
};

// Ô CHỌN MODE CHẠY BẰNG BÂY GIỜ (Thay đổi biến này để chọn Test A, B, C)
#define ACTIVE_PHASE3B_TEST_MODE   TEST_MODE_ROTATE_LEFT

enum CalibTestState {
  CALIB_INIT,
  CALIB_WAITING_COUNTDOWN, // Chờ 5.0s countdown đứng yên tại vạch xuất phát
  CALIB_POSE_RUNNING,      // Chạy bám theo Test Mode trong 3.0s
  CALIB_COASTING,          // Ngắt PWM = 0 hoàn toàn, tiếp tục tích phân Pose trôi quán tính cho tới khi dừng hẳn
  CALIB_TEST_COMPLETE      // Bánh dừng hẳn, chốt số liệu, lưu Flash NVS & in kết quả định kỳ 1s
};

/**
 * @brief Khởi tạo ngắt đếm xung cho Phase 3A Odometry Calibration
 */
void setupOdometryInterrupts();

/**
 * @brief Chạy máy trạng thái Odometry Calibration Phase 3A
 */
void runOdometryCalibration(MotorDriver& motors, SpeedController& leftCtrl, SpeedController& rightCtrl);

#endif // ODOMETRY_CALIBRATION_H
