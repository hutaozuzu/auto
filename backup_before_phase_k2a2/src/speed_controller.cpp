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
      _max_kick_cycles(4),        // 4 chu kỳ x 50ms = 200ms giới hạn kick tối đa
      _kick_pwm(84),              // Mức PWM vượt ma sát tĩnh thực nghiệm tải sàn 680g
      _kick_cooldown(0),
      _start_attempt_count(0),    // Đếm số lần thử kick
      _max_start_attempts(3),     // Giới hạn 3 lần kick trước khi latch an toàn
      _stall_latched(false),      // Khóa kẹt động cơ an toàn
      _zero_target_cycles(0),
      _sustain_hold_active(false),
      _sustain_hold_cycles(0),
      _max_sustain_hold_cycles(6), // 6 chu kỳ x 50ms = 300ms sustain-hold
      _sustain_pwm(74) {
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

    // Hiệu chuẩn Feedforward dải tốc độ thấp (8 - 45 RPM) theo tải trọng sàn 680g (Phase M2.6E)
    if (target_rpm <= rpm_table[0]) {
        float min_floor_pwm = _is_left_side ? 64.0f : 66.0f;
        float ratio = target_rpm / rpm_table[0];
        float ff = min_floor_pwm + ratio * (90.0f - min_floor_pwm);
        return (uint8_t)constrain(ff, 0.0f, (float)_pwm_max);
    }

    // Trường hợp vượt quá mốc RPM thực nghiệm lớn nhất
    if (target_rpm >= rpm_table[NUM_FF_POINTS - 1]) {
        return (uint8_t)FF_PWM[NUM_FF_POINTS - 1];
    }

    // Nội suy tuyến tính giữa các điểm thực nghiệm
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

    // NẾU ĐANG BỊ KHÓA STALL LATCH -> ACTUATOR OUTPUT = 0
    if (_stall_latched) {
        reset();
        resetKick();
        if (out_p_term) *out_p_term = 0.0f;
        if (out_i_term) *out_i_term = 0.0f;
        if (out_pwm_ff) *out_pwm_ff = 0;
        return 0;
    }

    // --- Bounded Startup Kick & Sustain Transition Logic (Phase M2.6E) ---
    if (_measured_rpm > 1.0f || raw_pulse_delta > 0) {
        // Đã phát hiện chuyển động thực tế từ Encoder
        if (_kick_active) {
            _kick_active = false;
            _kick_cycles = 0;
            _kick_cooldown = 0;
            _sustain_hold_active = true;
            _sustain_hold_cycles = 0;
            _start_attempt_count = 0; // Khởi động thành công
        }
    } else {
        // Bánh xe đang đứng yên trong khi Target > 0
        if (!_kick_active && !_sustain_hold_active && _kick_cooldown == 0) {
            if (_start_attempt_count < _max_start_attempts) {
                _kick_active = true;
                _kick_cycles = 0;
                _start_attempt_count++;
            } else {
                // Quá 3 lần kick (600ms) không có xung -> Khóa STALL_LATCH an toàn
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
                _kick_active = false;
                if (_start_attempt_count >= _max_start_attempts) {
                    _stall_latched = true;
                    _integral = 0.0f;
                    if (out_p_term) *out_p_term = 0.0f;
                    if (out_i_term) *out_i_term = 0.0f;
                    if (out_pwm_ff) *out_pwm_ff = 0;
                    return 0;
                } else {
                    _kick_cooldown = 4; // 200ms cooldown trước khi thử lần tiếp
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

    // 1. Tính toán Feedforward PWM từ bảng thực nghiệm tải sàn
    uint8_t pwm_ff = computeFeedforward(_target_rpm);
    if (out_pwm_ff) *out_pwm_ff = pwm_ff;

    // 2. Tính toán Sai số (Error)
    float error = _target_rpm - _measured_rpm;

    // 3. Tính toán P-Term
    float p_term = _kp * error;
    if (out_p_term) *out_p_term = p_term;

    // 4. Tính toán I-Term với Anti-Windup Clamping
    float potential_integral = _integral + (error * dt_seconds);
    float potential_i_term = _ki * potential_integral;

    float total_pwm = (float)pwm_ff + p_term + potential_i_term;

    if (total_pwm < (float)_pwm_max && total_pwm > (float)_pwm_min) {
        _integral = potential_integral;
    }
    _integral = constrain(_integral, -60.0f, 60.0f);

    float i_term = _ki * _integral;
    if (out_i_term) *out_i_term = i_term;

    // 5. Tổng hợp lệnh PWM cuối cùng
    float final_pwm_float = (float)pwm_ff + p_term + i_term;

    // 6. Sàn Sustain thích ứng theo Target và Động cơ (Phase M2.6E)
    uint8_t min_sustain_base = _is_left_side ? 72 : 76;
    uint8_t adaptive_sustain = (uint8_t)fmaxf((float)min_sustain_base, (float)pwm_ff + 4.0f);

    // Áp dụng Startup Kick PWM hoặc Sustain-Hold PWM
    if (_kick_active && final_pwm_float < (float)_kick_pwm) {
        final_pwm_float = (float)_kick_pwm;
    } else if (_sustain_hold_active && final_pwm_float < (float)adaptive_sustain) {
        final_pwm_float = (float)adaptive_sustain;
    }

    uint8_t final_pwm = (uint8_t)constrain(final_pwm_float, (float)_pwm_min, (float)_pwm_max);

    _prev_error = error;
    return final_pwm;
}
