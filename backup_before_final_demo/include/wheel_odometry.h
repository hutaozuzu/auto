#ifndef WHEEL_ODOMETRY_H
#define WHEEL_ODOMETRY_H

#include <Arduino.h>

/**
 * @brief Class WheelOdometry tính toán Động học Differential-Drive 2WD
 * Chịu trách nhiệm tích phân vị trí (x, y, theta) dựa thuần túy trên xung ngắt đĩa quang FC-03.
 * Độc lập 100% với ROS2, Wi-Fi, IMU, LiDAR hay Nav2.
 */
class WheelOdometry {
private:
    float _wheel_diameter_m;
    float _wheel_separation_m;
    uint16_t _pulses_per_rev;

    float _distance_per_pulse;

    // Pose States (SI Units: meters, radians)
    float _x_m;
    float _y_m;
    float _theta_rad;       // Normalized [-PI, +PI]
    float _theta_rad_total; // Continuous un-wrapped angle

public:
    /**
     * @brief Khởi tạo WheelOdometry
     * @param wheel_diameter_m Đường kính bánh xe (m) - Mặc định 0.065m
     * @param wheel_separation_m Khoảng cách giữa 2 tâm bánh xe (m) - Mặc định 0.160m
     * @param pulses_per_rev Số xung/vòng (PPR) - Mặc định 38
     */
    WheelOdometry(float wheel_diameter_m = 0.065f, float wheel_separation_m = 0.160f, uint16_t pulses_per_rev = 38);

    /**
     * @brief Cấu hình kích thước cơ khí thực tế
     */
    void setGeometry(float wheel_diameter_m, float wheel_separation_m);

    /**
     * @brief Reset hoàn toàn pose về 0 (x=0, y=0, theta=0)
     */
    void reset();

    /**
     * @brief Cập nhật Pose dựa trên delta xung CÓ DẤU (Signed Delta Pulses) của 2 bánh
     * @param delta_left_pulses Số xung có dấu bánh Trái trong chu kỳ (Âm nếu Lùi, Dương nếu Tiến)
     * @param delta_right_pulses Số xung có dấu bánh Phải trong chu kỳ (Âm nếu Lùi, Dương nếu Tiến)
     * @param out_delta_s Con trỏ nhận delta_s (m)
     * @param out_delta_theta Con trỏ nhận delta_theta (rad)
     */
    void update(int32_t delta_left_pulses, int32_t delta_right_pulses, float* out_delta_s = nullptr, float* out_delta_theta = nullptr);

    // Getters
    float getX() const { return _x_m; }
    float getY() const { return _y_m; }
    float getThetaRad() const { return _theta_rad; }
    float getThetaDeg() const { return _theta_rad * (180.0f / PI); }
    float getThetaRadTotal() const { return _theta_rad_total; }
    float getThetaDegTotal() const { return _theta_rad_total * (180.0f / PI); }
    float getDistancePerPulse() const { return _distance_per_pulse; }
};

#endif // WHEEL_ODOMETRY_H
