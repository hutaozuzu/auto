#include "speed_controller.h"

// Định nghĩa mảng hằng số tĩnh
constexpr float SpeedController::FF_PWM[7];
constexpr float SpeedController::FF_LEFT_RPM[7];
constexpr float SpeedController::FF_RIGHT_RPM[7];

SpeedController::SpeedController(bool is_left, float kp, float ki, uint16_t pwm_min, uint16_t pwm_max)
    : _is_left_side(is_left),
      _kp(kp),
      _ki(ki),
      _kd(0.0f), // Khóa cứng Kd = 0
      _integral(0.0f),
      _prev_error(0.0f),
      _target_rpm(0.0f),
      _measured_rpm(0.0f),
      _pwm_min(pwm_min),
      _pwm_max(pwm_max),
      _kick_active(false),
      _kick_cycles(0),
      _max_kick_cycles(3),   // 3 chu kỳ x 50ms = 150ms giới hạn kick tối đa
      _kick_pwm(74),         // Mức PWM vượt ma sát tĩnh thực nghiệm
      _kick_cooldown(0) {
}

void SpeedController::setGains(float kp, float ki) {
    _kp = kp;
    _ki = ki;
    _kd = 0.0f; // Luôn đảm bảo Kd = 0
}

void SpeedController::setTargetRPM(float target_rpm) {
    if (target_rpm < 0.0f) target_rpm = 0.0f; // Phase 2C ưu tiên tốc độ tiến
    _target_rpm = target_rpm;
    if (_target_rpm == 0.0f) {
        reset();
        resetKick();
    }
}

void SpeedController::reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
    _measured_rpm = 0.0f;
}

uint8_t SpeedController::computeFeedforward(float target_rpm) {
    if (target_rpm <= 0.0f) return 0;

    const float* rpm_table = _is_left_side ? FF_LEFT_RPM : FF_RIGHT_RPM;

    // Trường hợp nhỏ hơn mốc RPM thực nghiệm đầu tiên (49.74 / 34.89)
    // Áp dụng offset PWM giữ quay tối thiểu (48 PWM) + nội suy lên mốc FF_PWM[0] (65 PWM)
    // để khắc phục ma sát tĩnh ở dải tốc độ thấp (8 - 40 RPM) mà không thay đổi Target RPM.
    if (target_rpm <= rpm_table[0]) {
        float min_hold_pwm = 48.0f;
        float ratio = target_rpm / rpm_table[0];
        float ff = min_hold_pwm + ratio * (FF_PWM[0] - min_hold_pwm);
        return (uint8_t)constrain(ff, 0.0f, (float)_pwm_max);
    }

    // Trường hợp vượt quá mốc RPM thực nghiệm lớn nhất
    if (target_rpm >= rpm_table[NUM_FF_POINTS - 1]) {
        return (uint8_t)FF_PWM[NUM_FF_POINTS - 1];
    }

    // Nội suy tuyến tính (Piecewise Linear Interpolation) giữa các điểm thực nghiệm
    for (int i = 0; i < NUM_FF_POINTS - 1; i++) {
        if (target_rpm >= rpm_table[i] && target_rpm <= rpm_table[i + 1]) {
            float slope = (FF_PWM[i + 1] - FF_PWM[i]) / (rpm_table[i + 1] - rpm_table[i]);
            float ff = FF_PWM[i] + slope * (target_rpm - rpm_table[i]);
            return (uint8_t)constrain(ff, 0.0f, (float)_pwm_max);
        }
    }

    return 0;
}

uint8_t SpeedController::update(float measured_rpm, float dt_seconds, float* out_p_term, float* out_i_term, uint8_t* out_pwm_ff) {
    _measured_rpm = measured_rpm;

    // NẾU TARGET = 0 -> DỪNG KHẨN CẤP VÀ RESET INTEGRAL & KICK
    if (_target_rpm <= 0.0f) {
        reset();
        resetKick();
        if (out_p_term) *out_p_term = 0.0f;
        if (out_i_term) *out_i_term = 0.0f;
        if (out_pwm_ff) *out_pwm_ff = 0;
        return 0;
    }

    // --- Bounded Startup Kick Logic (Phase M2) ---
    // Kiểm tra xem bánh xe đã bắt đầu quay thực tế hay chưa
    if (_measured_rpm > 1.0f) {
        // Đã có chuyển động thực tế -> Lập tức thoát kick và trở lại điều khiển PI + FF bình thường
        _kick_active = false;
        _kick_cycles = 0;
        _kick_cooldown = 0;
    } else {
        // Bánh xe đang đứng yên trong khi Target > 0
        if (!_kick_active && _kick_cooldown == 0) {
            _kick_active = true;
            _kick_cycles = 0;
        }

        if (_kick_active) {
            _kick_cycles++;
            if (_kick_cycles >= _max_kick_cycles) {
                // Đã hết thời gian kick tối đa (150ms) -> Ngắt kick an toàn để chống kẹt nóng động cơ
                _kick_active = false;
                _kick_cooldown = 6; // 300ms cooldown
            }
        } else if (_kick_cooldown > 0) {
            _kick_cooldown--;
        }
    }

    // 1. Tính toán Feedforward PWM từ bảng thực nghiệm
    uint8_t pwm_ff = computeFeedforward(_target_rpm);
    if (out_pwm_ff) *out_pwm_ff = pwm_ff;

    // 2. Tính toán Sai số (Error)
    float error = _target_rpm - _measured_rpm;

    // 3. Tính toán P-Term
    float p_term = _kp * error;
    if (out_p_term) *out_p_term = p_term;

    // 4. Tính toán I-Term với Anti-Windup (Conditional Integration)
    float potential_integral = _integral + (error * dt_seconds);
    float potential_i_term = _ki * potential_integral;

    // Tổng đầu ra PWM dự kiến
    float total_pwm = (float)pwm_ff + p_term + potential_i_term;

    // Anti-Windup Clamping: Chỉ cộng dồn tích phân nếu tổng PWM chưa chạm trần _pwm_max hoặc bão hòa
    if (total_pwm < (float)_pwm_max && total_pwm > (float)_pwm_min) {
        _integral = potential_integral;
    }
    // Giới hạn vùng tích phân không vượt quá biên ±50 PWM
    _integral = constrain(_integral, -50.0f, 50.0f);

    float i_term = _ki * _integral;
    if (out_i_term) *out_i_term = i_term;

    // 5. Tổng hợp lệnh PWM cuối cùng
    float final_pwm_float = (float)pwm_ff + p_term + i_term;

    // Áp dụng Startup Kick PWM nếu kick đang active và PWM tính toán thấp hơn kick_pwm
    if (_kick_active && final_pwm_float < (float)_kick_pwm) {
        final_pwm_float = (float)_kick_pwm;
    }

    uint8_t final_pwm = (uint8_t)constrain(final_pwm_float, (float)_pwm_min, (float)_pwm_max);

    _prev_error = error;
    return final_pwm;
}
