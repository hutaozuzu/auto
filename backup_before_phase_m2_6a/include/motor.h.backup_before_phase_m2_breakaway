#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/**
 * @brief Class điều khiển Motor Driver TB6612FNG dành cho Robot 2WD.
 * Hỗ trợ điều khiển hướng và tốc độ PWM độc lập cho Động cơ Trái (Motor A) và Động cơ Phải (Motor B).
 */
class MotorDriver {
private:
    uint8_t _pwma, _ain1, _ain2;
    uint8_t _pwmb, _bin1, _bin2;
    uint8_t _stby;

    // Cấu hình LEDC (Hardware PWM) của ESP32
    uint32_t _pwm_freq;
    uint8_t _pwm_resolution;
    uint8_t _channel_a;
    uint8_t _channel_b;

    // Giới hạn tốc độ PWM
    uint16_t _max_pwm;

public:
    /**
     * @brief Khởi tạo MotorDriver với sơ đồ chân GPIO của ESP32
     * 
     * @param pwma Chân PWM động cơ trái (Motor A)
     * @param ain1 Chân hướng IN1 động cơ trái
     * @param ain2 Chân hướng IN2 động cơ trái
     * @param pwmb Chân PWM động cơ phải (Motor B)
     * @param bin1 Chân hướng IN1 động cơ phải
     * @param bin2 Chân hướng IN2 động cơ phải
     * @param stby Chân Standby điều khiển bật/tắt TB6612FNG
     * @param pwm_freq Tần số PWM (Mặc định 5000Hz)
     * @param pwm_resolution Độ phân giải PWM (Mặc định 8-bit, 0-255)
     * @param channel_a Kênh LEDC PWM A (Mặc định channel 0)
     * @param channel_b Kênh LEDC PWM B (Mặc định channel 1)
     */
    MotorDriver(uint8_t pwma, uint8_t ain1, uint8_t ain2,
                uint8_t pwmb, uint8_t bin1, uint8_t bin2,
                uint8_t stby,
                uint32_t pwm_freq = 5000,
                uint8_t pwm_resolution = 8,
                uint8_t channel_a = 0,
                uint8_t channel_b = 1);

    /**
     * @brief Cấu hình GPIO và khởi tạo LEDC PWM channels
     */
    void begin();

    /**
     * @brief Bật hoặc tắt trạng thái Standby của TB6612FNG
     * @param enable true để bật driver, false để tắt (Standby)
     */
    void setStandby(bool enable);

    /**
     * @brief Điều khiển Động cơ Trái (Motor A)
     * @param speed Tốc độ từ -255 đến 255 (Dương: Tiến, Âm: Lùi, 0: Dừng)
     */
    void setLeftMotor(int16_t speed);

    /**
     * @brief Điều khiển Động cơ Phải (Motor B)
     * @param speed Tốc độ từ -255 đến 255 (Dương: Tiến, Âm: Lùi, 0: Dừng)
     */
    void setRightMotor(int16_t speed);

    /**
     * @brief Đặt tốc độ đồng thời cho cả 2 động cơ (API chuẩn cho Odometry / PID / cmd_vel)
     * @param left_speed Tốc độ động cơ trái (-255 đến 255)
     * @param right_speed Tốc độ động cơ phải (-255 đến 255)
     */
    void setSpeed(int16_t left_speed, int16_t right_speed);

    /**
     * @brief Cho robot tiến thẳng
     * @param speed Tốc độ PWM (0 đến 255)
     */
    void forward(uint8_t speed);

    /**
     * @brief Cho robot lùi thẳng
     * @param speed Tốc độ PWM (0 đến 255)
     */
    void backward(uint8_t speed);

    /**
     * @brief Cho robot rẽ trái (Động cơ trái lùi/dừng, động cơ phải tiến)
     * @param speed Tốc độ PWM (0 đến 255)
     */
    void turnLeft(uint8_t speed);

    /**
     * @brief Cho robot rẽ phải (Động cơ trái tiến, động cơ phải lùi/dừng)
     * @param speed Tốc độ PWM (0 đến 255)
     */
    void turnRight(uint8_t speed);

    /**
     * @brief Dừng cả 2 động cơ ngay lập tức (Coast stop / Brake)
     */
    void stop();
};

#endif // MOTOR_H
