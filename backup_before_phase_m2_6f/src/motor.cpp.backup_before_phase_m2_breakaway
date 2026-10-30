#include "motor.h"

MotorDriver::MotorDriver(uint8_t pwma, uint8_t ain1, uint8_t ain2,
                         uint8_t pwmb, uint8_t bin1, uint8_t bin2,
                         uint8_t stby,
                         uint32_t pwm_freq,
                         uint8_t pwm_resolution,
                         uint8_t channel_a,
                         uint8_t channel_b)
    : _pwma(pwma), _ain1(ain1), _ain2(ain2),
      _pwmb(pwmb), _bin1(bin1), _bin2(bin2),
      _stby(stby),
      _pwm_freq(pwm_freq),
      _pwm_resolution(pwm_resolution),
      _channel_a(channel_a),
      _channel_b(channel_b) {
    _max_pwm = (1 << _pwm_resolution) - 1; // 255 đối với 8-bit
}

void MotorDriver::begin() {
    // Cấu hình các chân điều khiển Hướng và STBY là OUTPUT
    pinMode(_ain1, OUTPUT);
    pinMode(_ain2, OUTPUT);
    pinMode(_bin1, OUTPUT);
    pinMode(_bin2, OUTPUT);
    pinMode(_stby, OUTPUT);

    // Mặc định dừng hướng
    digitalWrite(_ain1, LOW);
    digitalWrite(_ain2, LOW);
    digitalWrite(_bin1, LOW);
    digitalWrite(_bin2, LOW);

    // Bật Standby (STBY HIGH để kích hoạt TB6612FNG)
    setStandby(true);

    // Cấu hình Hardware PWM (LEDC) của ESP32 (Hỗ trợ tương thích cả ESP-IDF v4 và v5/Arduino core 2.x & 3.x)
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(_pwma, _pwm_freq, _pwm_resolution);
    ledcAttach(_pwmb, _pwm_freq, _pwm_resolution);
#else
    ledcSetup(_channel_a, _pwm_freq, _pwm_resolution);
    ledcAttachPin(_pwma, _channel_a);

    ledcSetup(_channel_b, _pwm_freq, _pwm_resolution);
    ledcAttachPin(_pwmb, _channel_b);
#endif

    stop();
}

void MotorDriver::setStandby(bool enable) {
    digitalWrite(_stby, enable ? HIGH : LOW);
}

void MotorDriver::setLeftMotor(int16_t speed) {
    // Kẹp giá trị tốc độ trong khoảng [-255, 255]
    if (speed > (int16_t)_max_pwm) speed = _max_pwm;
    if (speed < -(int16_t)_max_pwm) speed = -_max_pwm;

    if (speed > 0) {
        // Tiến (Forward)
        digitalWrite(_ain1, LOW);
        digitalWrite(_ain2, HIGH);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pwma, speed);
#else
        ledcWrite(_channel_a, speed);
#endif
    } else if (speed < 0) {
        // Lùi (Reverse)
        digitalWrite(_ain1, HIGH);
        digitalWrite(_ain2, LOW);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pwma, -speed);
#else
        ledcWrite(_channel_a, -speed);
#endif
    } else {
        // Dừng (Coast Stop)
        digitalWrite(_ain1, LOW);
        digitalWrite(_ain2, LOW);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pwma, 0);
#else
        ledcWrite(_channel_a, 0);
#endif
    }
}

void MotorDriver::setRightMotor(int16_t speed) {
    // Kẹp giá trị tốc độ trong khoảng [-255, 255]
    if (speed > (int16_t)_max_pwm) speed = _max_pwm;
    if (speed < -(int16_t)_max_pwm) speed = -_max_pwm;

    if (speed > 0) {
        // Tiến (Forward)
        digitalWrite(_bin1, HIGH);
        digitalWrite(_bin2, LOW);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pwmb, speed);
#else
        ledcWrite(_channel_b, speed);
#endif
    } else if (speed < 0) {
        // Lùi (Reverse)
        digitalWrite(_bin1, LOW);
        digitalWrite(_bin2, HIGH);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pwmb, -speed);
#else
        ledcWrite(_channel_b, -speed);
#endif
    } else {
        // Dừng (Coast Stop)
        digitalWrite(_bin1, LOW);
        digitalWrite(_bin2, LOW);
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(_pwmb, 0);
#else
        ledcWrite(_channel_b, 0);
#endif
    }
}

void MotorDriver::setSpeed(int16_t left_speed, int16_t right_speed) {
    setLeftMotor(left_speed);
    setRightMotor(right_speed);
}

void MotorDriver::forward(uint8_t speed) {
    setSpeed(speed, speed);
}

void MotorDriver::backward(uint8_t speed) {
    setSpeed(-speed, -speed);
}

void MotorDriver::turnLeft(uint8_t speed) {
    setSpeed(-speed, speed);
}

void MotorDriver::turnRight(uint8_t speed) {
    setSpeed(speed, -speed);
}

void MotorDriver::stop() {
    setSpeed(0, 0);
}
