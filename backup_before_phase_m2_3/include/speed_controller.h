#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Class điều khiển tốc độ PI Vòng kín (Closed-Loop PI Controller) 
 * kết hợp Piecewise Linear Feedforward, Anti-Windup, Bounded Startup Kick và Stall Latch.
 */
class SpeedController {
private:
    float _kp;
    float _ki;
    float _kd;  // Khóa cứng Kd = 0 theo yêu cầu thiết kế chống nhiễu đĩa 38 PPR

    float _integral;
    float _prev_error;
    float _target_rpm;
    float _measured_rpm;

    uint16_t _pwm_min;
    uint16_t _pwm_max;
    bool _is_left_side;

    // Bounded Startup Kick & Stall Latch state variables (Phase M2.2)
    bool _kick_active;
    uint8_t _kick_cycles;
    uint8_t _max_kick_cycles;     // 3 chu kỳ x 50ms = 150ms
    uint8_t _kick_pwm;            // Mức PWM vượt ma sát tĩnh
    uint8_t _kick_cooldown;       // 6 chu kỳ x 50ms = 300ms
    uint8_t _start_attempt_count; // Số lần kick đã thực hiện trong command episode hiện tại
    uint8_t _max_start_attempts;  // Giới hạn 2 lần kick tối đa
    bool _stall_latched;          // Cờ khóa kẹt cứng (Stall Latch)
    uint8_t _zero_target_cycles;  // Đếm chu kỳ target=0 liên tục để rearm episode

    // Piecewise Linear Feedforward Lookup Table dựa trên dữ liệu lặp 5 lần Phase 2B.1 thực tế
    // Mảng dữ liệu PWM thực nghiệm: {65, 90, 120, 150, 180, 210, 255}
    static const int NUM_FF_POINTS = 7;
    static constexpr float FF_PWM[7] = {65.0f, 90.0f, 120.0f, 150.0f, 180.0f, 210.0f, 255.0f};

    // RPM thực nghiệm Motor Trái Phase 2C.5 Final Lock (Target 80 RPM -> FF 132 PWM)
    static constexpr float FF_LEFT_RPM[7] = {49.74f, 62.84f, 70.00f, 106.00f, 151.26f, 177.79f, 181.89f};

    // RPM thực nghiệm Motor Phải Phase 2C.5 Final Lock (Target 80 RPM -> FF 136 PWM)
    static constexpr float FF_RIGHT_RPM[7] = {34.89f, 65.05f, 70.00f, 104.00f, 138.00f, 167.21f, 217.89f};

public:
    /**
     * @brief Khởi tạo SpeedController
     * 
     * @param is_left true nếu là Motor Trái, false nếu là Motor Phải
     * @param kp Kp ban đầu
     * @param ki Ki ban đầu
     * @param pwm_min Giới hạn PWM thấp nhất (Mặc định 0)
     * @param pwm_max Giới hạn PWM cao nhất cho bài tune (Mặc định 210)
     */
    SpeedController(bool is_left, float kp = 0.5f, float ki = 0.2f, uint16_t pwm_min = 0, uint16_t pwm_max = 210);

    /**
     * @brief Thiết lập thông số Gains cho bộ điều khiển
     */
    void setGains(float kp, float ki);

    /**
     * @brief Đặt tốc độ RPM mục tiêu (Target RPM)
     */
    void setTargetRPM(float target_rpm);

    /**
     * @brief Lấy tốc độ RPM mục tiêu hiện tại
     */
    float getTargetRPM() const { return _target_rpm; }

    /**
     * @brief Cấu hình thông số Bounded Startup Kick
     */
    void setKickParams(uint8_t kick_pwm = 74, uint8_t max_cycles = 3, uint8_t max_attempts = 2) {
        _kick_pwm = kick_pwm;
        _max_kick_cycles = max_cycles;
        _max_start_attempts = max_attempts;
    }

    /**
     * @brief Reset trạng thái Startup Kick
     */
    void resetKick() {
        _kick_active = false;
        _kick_cycles = 0;
        _kick_cooldown = 0;
    }

    /**
     * @brief Kiểm tra xem Startup Kick có đang được kích hoạt hay không
     */
    bool isKickActive() const { return _kick_active; }

    /**
     * @brief Kiểm tra xem Motor có đang bị khóa Stall Latch hay không
     */
    bool isStallLatched() const { return _stall_latched; }

    /**
     * @brief Lấy số lần thử kick trong episode hiện tại
     */
    uint8_t getStartAttempts() const { return _start_attempt_count; }

    /**
     * @brief Reset Stall Latch thủ công
     */
    void resetStall() {
        _stall_latched = false;
        _start_attempt_count = 0;
        _zero_target_cycles = 0;
    }

    /**
     * @brief Tính toán Feedforward PWM từ bảng dữ liệu thực nghiệm Phase 2B.1
     */
    uint8_t computeFeedforward(float target_rpm);

    /**
     * @brief Tính toán tín hiệu đầu ra PWM vòng kín (PI Output + Feedforward + Bounded Startup Kick + Stall Latch)
     * 
     * @param measured_rpm Vận tốc thực tế vừa đo được từ Encoder
     * @param raw_pulse_delta Số xung encoder đo được trong chu kỳ
     * @param dt_seconds Thời gian chu kỳ (seconds)
     * @param out_p_term Con trỏ nhận giá trị P-Term để logging
     * @param out_i_term Con trỏ nhận giá trị I-Term để logging
     * @param out_pwm_ff Con trỏ nhận giá trị PWM Feedforward để logging
     * @return uint8_t Giá trị lệnh PWM cuối cùng (0 - pwm_max)
     */
    uint8_t update(float measured_rpm, uint32_t raw_pulse_delta, float dt_seconds, float* out_p_term, float* out_i_term, uint8_t* out_pwm_ff);

    /**
     * @brief Reset hoàn toàn bộ tích phân Integral và Error khi Target = 0
     */
    void reset();
};

#endif // SPEED_CONTROLLER_H
