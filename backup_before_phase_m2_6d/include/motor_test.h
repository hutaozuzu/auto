#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "motor.h"

/**
 * @brief Kịch bản kiểm thử Động cơ Non-blocking thời gian thực.
 * Chạy song song không gây nghẽn luồng UART LiDAR hoặc micro-ROS.
 * 
 * @param motors Tham chiếu đến đối tượng MotorDriver
 */
void runNonBlockingMotorTest(MotorDriver& motors);

#endif // MOTOR_TEST_H
