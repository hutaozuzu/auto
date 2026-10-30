#include "wheel_odometry.h"

WheelOdometry::WheelOdometry(float wheel_diameter_m, float wheel_separation_m, uint16_t pulses_per_rev)
    : _wheel_diameter_m(wheel_diameter_m),
      _wheel_separation_m(wheel_separation_m),
      _pulses_per_rev(pulses_per_rev),
      _x_m(0.0f),
      _y_m(0.0f),
      _theta_rad(0.0f),
      _theta_rad_total(0.0f) {
    setGeometry(_wheel_diameter_m, _wheel_separation_m);
}

void WheelOdometry::setGeometry(float wheel_diameter_m, float wheel_separation_m) {
    _wheel_diameter_m = wheel_diameter_m;
    _wheel_separation_m = wheel_separation_m;
    float wheel_circumference = PI * _wheel_diameter_m;
    _distance_per_pulse = wheel_circumference / (float)_pulses_per_rev;
}

void WheelOdometry::reset() {
    _x_m = 0.0f;
    _y_m = 0.0f;
    _theta_rad = 0.0f;
    _theta_rad_total = 0.0f;
}

void WheelOdometry::update(int32_t delta_left_pulses, int32_t delta_right_pulses, float* out_delta_s, float* out_delta_theta) {
    // 1. Tính quãng đường từng bánh di chuyển trong chu kỳ delta
    float delta_left_dist = (float)delta_left_pulses * _distance_per_pulse;
    float delta_right_dist = (float)delta_right_pulses * _distance_per_pulse;

    // 2. Thuật toán Differential Drive Kinematics (SI Units: m, rad)
    float delta_s = (delta_right_dist + delta_left_dist) / 2.0f;
    float delta_theta = (delta_right_dist - delta_left_dist) / _wheel_separation_m;

    if (out_delta_s) *out_delta_s = delta_s;
    if (out_delta_theta) *out_delta_theta = delta_theta;

    // 3. Tích phân Midpoint Integration (Tăng độ chính xác hơn Euler Euler)
    float theta_mid = _theta_rad + (delta_theta / 2.0f);

    _x_m += delta_s * cosf(theta_mid);
    _y_m += delta_s * sinf(theta_mid);
    _theta_rad += delta_theta;
    _theta_rad_total += delta_theta;

    // 4. Normalize Theta về khoảng [-PI, +PI]
    while (_theta_rad > PI)  _theta_rad -= 2.0f * PI;
    while (_theta_rad < -PI) _theta_rad += 2.0f * PI;
}
