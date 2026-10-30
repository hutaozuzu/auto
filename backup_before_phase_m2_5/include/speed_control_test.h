#ifndef SPEED_CONTROL_TEST_H
#define SPEED_CONTROL_TEST_H

#include <Arduino.h>
#include "motor.h"
#include "speed_controller.h"

// ==========================================
// CẤU HÌNH GPIO ENCODER FC-03 & CHU KỲ ĐIỀU KHIỂN
// ==========================================
#define ENCODER_LEFT_PIN       34   // GPI 34: Input Only
#define ENCODER_RIGHT_PIN      35   // GPI 35: Input Only

#define PULSES_PER_REV         38   // Thực nghiệm 38 xung/vòng
#define CONTROL_PERIOD_MS      200  // Chu kỳ vòng lặp PI control cố định: 200 ms (5Hz)

#define STALL_SAFETY_TIMEOUT_MS 1500 // Cảnh báo/ngắt an toàn nếu motor có target > 0 mà 1.5s không nhận được xung nào
#define MAX_PHYSICAL_RPM_LIMIT  400.0f // Ngưỡng vận tốc tối đa hợp lý về mặt vật lý (dùng làm Sanity Check)

// GAINS TEST BAN ĐẦU (INITIAL TEST VALUES)
#define INITIAL_KP_LEFT        0.4f
#define INITIAL_KI_LEFT        0.1f

#define INITIAL_KP_RIGHT       0.4f
#define INITIAL_KI_RIGHT       0.1f

// Các kịch bản thử nghiệm Phase 2C.4 Open-Loop Sweep
enum ControlTestState {
  CTRL_INIT,
  CTRL_SWEEP_RIGHT_OPEN_LOOP, // Open-Loop Sweep Motor Phải ở các mức PWM: 123, 126, 129, 132, 135, 138, 141
  CTRL_DONE
};

/**
 * @brief Khởi tạo ngắt ISR đếm xung
 */
void setupSpeedControlInterrupts();

/**
 * @brief Máy trạng thái thực thi thử nghiệm Phase 2C.1 Re-Validation
 */
void runClosedLoopSpeedTest(MotorDriver& motors, SpeedController& leftCtrl, SpeedController& rightCtrl);

#endif // SPEED_CONTROL_TEST_H
