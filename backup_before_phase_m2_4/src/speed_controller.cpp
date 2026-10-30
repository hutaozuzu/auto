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
      _max_kick_cycles(3),        // 3 chu kỳ x 50ms = 150ms giới hạn kick tối đa
      _kick_pwm(78),              // Mức PWM vượt ma sát tĩnh thực nghiệm
      _kick_cooldown(0),
      _start_attempt_count(0),    // Đếm số lần thử kick
      _max_start_attempts(2),     // Giới hạn tối đa 2 lần kick cho mỗi command episode
      _stall_latched(false),      // Khóa kẹt động cơ an toàn
      _zero_target_cycles(0),
      _sustain_hold_active(false),
      _sustain_hold_cycles(0),
      _max_sustain_hold_cycles(5), // 5 chu kỳ x 50ms = 250ms sustain-hold
      _sustain_pwm(70) {          // Mức PWM duy trì sau kick
}

void SpeedController::setGains(float kp, float ki) {
    _kp = kp;
    _ki = ki;
    _kd = 0.0f; // Luôn đảm bảo Kd = 0
}

void SpeedController::setTargetRPM(float target_rpm) {
    if (target_rpm < 0.0f) target_rpm = 0.0f; // Tốc độ tiến (Magnitude)
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

uint8_t SpeedController::update(float measured_rpm, uint32_t raw_pulse_delta, float dt_seconds, float* out_p_term, float* out_i_term, uint8_t* out_pwm_ff) {
    _measured_rpm = measured_rpm;

    // NẾU TARGET = 0 -> DỪNG VÀ REARM STALL LATCH NẾU TARGET DUY TRÌ 0 ĐỦ LÂU (300MS)
    if (_target_rpm <= 0.0f) {
        reset();
        resetKick();
        _zero_target_cycles++;
        if (_zero_target_cycles >= 6) { // 6 chu kỳ x 50ms = 300ms
            _stall_latched = false;
            _start_attempt_count = 0;
        }
        if (out_p_term) *out_p_term = 0.0f;
        if (out_i_term) *out_i_term = 0.0f;
        if (out_pwm_ff) *out_pwm_ff = 0;
        return 0;
    }

    // Target > 0: Reset bộ đếm chu kỳ target 0
    _zero_target_cycles = 0;

    // NẾU ĐANG BỊ KHÓA STALL LATCH -> ACTUATOR OUTPUT = 0, TUYỆT ĐỐI KHÔNG KICK LẠI
    if (_stall_latched) {
        reset();
        resetKick();
        if (out_p_term) *out_p_term = 0.0f;
        if (out_i_term) *out_i_term = 0.0f;
        if (out_pwm_ff) *out_pwm_ff = 0;
        return 0;
    }

    // --- Bounded Startup Kick & Sustain Transition Logic (Phase M2.3) ---
    if (_measured_rpm > 1.0f || raw_pulse_delta > 0) {
        // Đã phát hiện chuyển động thực tế từ Encoder
        if (_kick_active) {
            _kick_active = false;
            _kick_cycles = 0;
            _kick_cooldown = 0;
            _sustain_hold_active = true;
            _sustain_hold_cycles = 0;
            _start_attempt_count = 0; // Đã khởi động thành công, reset attempt count
        }
    } else {
        // Bánh xe đang đứng yên trong khi Target > 0
        if (!_kick_active && !_sustain_hold_active && _kick_cooldown == 0) {
            if (_start_attempt_count < _max_start_attempts) {
                _kick_active = true;
                _kick_cycles = 0;
                _start_attempt_count++;
            } else {
                // Đã vượt quá 2 lần kick mà vẫn không quay -> Khóa STALL_LATCH an toàn
                _stall_latched = true;
                _kick_active = false;
                _sustain_hold_active = false;
                _integral = 0.0f;
                if (out_p_term) *out_p_term = 0.0f;
                if (out_i_term) *out_i_term = 0.0f;
                if (out_pwm_ff) *out_pwm_ff = 0;
                return 0;
            }
        }

        if (_kick_active) {
            _kick_cycles++;
            if (_kick_cycles >= _max_kick_cycles) {
                // Đã hết 150ms kick -> Ngắt kick và chuyển sang cooldown 300ms
                _kick_active = false;
                if (_start_attempt_count >= _max_start_attempts) {
                    _stall_latched = true;
                    _integral = 0.0f;
                    if (out_p_term) *out_p_term = 0.0f;
                    if (out_i_term) *out_i_term = 0.0f;
                    if (out_pwm_ff) *out_pwm_ff = 0;
                    return 0;
                } else {
                    _kick_cooldown = 6; // 300ms cooldown trước khi thử lần 2
                }
            }
        } else if (_kick_cooldown > 0) {
            _kick_cooldown--;
        }
    }

    if (_sustain_hold_active) {
        _sustain_hold_cycles++;
        if (_sustain_hold_cycles >= _max_sustain_hold_cycles) {
            _sustain_hold_active = false;
        }
    }

    if (_stall_latched) {
        return 0;
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

    // Áp dụng Startup Kick PWM hoặc Sustain-Hold PWM
    if (_kick_active && final_pwm_float < (float)_kick_pwm) {
        final_pwm_float = (float)_kick_pwm;
    } else if (_sustain_hold_active && final_pwm_float < (float)_sustain_pwm) {
        final_pwm_float = (float)_sustain_pwm;
    }

    uint8_t final_pwm = (uint8_t)constrain(final_pwm_float, (float)_pwm_min, (float)_pwm_max);

    _prev_error = error;
    return final_pwm;
}
