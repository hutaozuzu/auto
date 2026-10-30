#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <Arduino.h>
#include <math.h>

enum DifferentialDriveStatus {
  DRIVE_STATUS_OK,
  DRIVE_STATUS_TIMEOUT,
  DRIVE_STATUS_DISCONNECTED,
  DRIVE_STATUS_STOPPED,
  DRIVE_STATUS_CLAMPED
};

class DifferentialDrive {
private:
  float _wheel_diameter_m;
  float _wheel_separation_m;

  float _max_wheel_rpm;
  float _max_linear_mps;
  float _max_angular_radps;
  float _min_reliable_rpm;
  float _min_actuation_rpm;

  uint32_t _cmd_timeout_ms;
  uint32_t _last_cmd_ms;

  float _last_linear_x;
  float _last_angular_z;

  float _target_rpm_left;
  float _target_rpm_right;

  DifferentialDriveStatus _status;

public:
  DifferentialDrive(float wheel_diameter_m = 0.065f,
                    float wheel_separation_m = 0.110f,
                    float max_wheel_rpm = 200.0f,
                    uint32_t cmd_timeout_ms = 500);

  /**
   * @brief Cập nhật lệnh vận tốc /cmd_vel mới từ ROS2
   */
  void updateCommand(float linear_x, float angular_z, uint32_t current_ms);

  /**
   * @brief Đánh giá an toàn (Timeout, Agent Disconnect) và tính toán Inverse Kinematics
   */
  void compute(uint32_t current_ms, bool micro_ros_connected);

  // Getters
  float getTargetRPMLeft() const { return _target_rpm_left; }
  float getTargetRPMRight() const { return _target_rpm_right; }

  int8_t getLeftDirectionSign() const {
    if (_target_rpm_left > 0.0f) return 1;
    if (_target_rpm_left < 0.0f) return -1;
    return 0;
  }

  int8_t getRightDirectionSign() const {
    if (_target_rpm_right > 0.0f) return 1;
    if (_target_rpm_right < 0.0f) return -1;
    return 0;
  }

  float getLeftRPM_Magnitude() const { return fabsf(_target_rpm_left); }
  float getRightRPM_Magnitude() const { return fabsf(_target_rpm_right); }

  float getLastLinearX() const { return _last_linear_x; }
  float getLastAngularZ() const { return _last_angular_z; }

  DifferentialDriveStatus getStatus() const { return _status; }
  const char* getStatusString() const;

  float getMaxLinearMPS() const { return _max_linear_mps; }
  float getMaxAngularRadPS() const { return _max_angular_radps; }
  float getMaxWheelRPM() const { return _max_wheel_rpm; }
  uint32_t getCmdAgeMs(uint32_t current_ms) const {
    return (current_ms >= _last_cmd_ms) ? (current_ms - _last_cmd_ms) : 0;
  }
};

#endif // DIFFERENTIAL_DRIVE_H
