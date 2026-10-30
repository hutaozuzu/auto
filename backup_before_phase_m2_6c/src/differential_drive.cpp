#include "differential_drive.h"

DifferentialDrive::DifferentialDrive(float wheel_diameter_m,
                                     float wheel_separation_m,
                                     float max_wheel_rpm,
                                     uint32_t cmd_timeout_ms)
    : _wheel_diameter_m(wheel_diameter_m),
      _wheel_separation_m(wheel_separation_m),
      _max_wheel_rpm(max_wheel_rpm),
      _cmd_timeout_ms(cmd_timeout_ms),
      _last_cmd_ms(0),
      _last_linear_x(0.0f),
      _last_angular_z(0.0f),
      _target_rpm_left(0.0f),
      _target_rpm_right(0.0f),
      _status(DRIVE_STATUS_STOPPED) {

  // Tính toán giới hạn vận tốc tuyến tính và vận tốc góc tối đa tương ứng với _max_wheel_rpm
  float circumference = M_PI * _wheel_diameter_m;
  _max_linear_mps = (_max_wheel_rpm * circumference) / 60.0f;
  _max_angular_radps = (2.0f * _max_linear_mps) / _wheel_separation_m;

  _min_reliable_rpm = 40.0f;  // Mốc tham chiếu độ tin cậy vòng kín (Telemetry reference)
  _min_actuation_rpm = 8.0f;   // Ngưỡng chấp nhận lệnh tối thiểu cho đĩa xung 38 PPR
}

void DifferentialDrive::updateCommand(float linear_x, float angular_z, uint32_t current_ms) {
  _last_linear_x = linear_x;
  _last_angular_z = angular_z;
  _last_cmd_ms = current_ms;
}

void DifferentialDrive::compute(uint32_t current_ms, bool micro_ros_connected) {
  // 1. Kiểm tra an toàn: micro-ROS rớt mạng -> STOP khẩn cấp
  if (!micro_ros_connected) {
    _target_rpm_left = 0.0f;
    _target_rpm_right = 0.0f;
    _status = DRIVE_STATUS_DISCONNECTED;
    return;
  }

  // 2. Kiểm tra an toàn: Timeout ngắt nhận lệnh /cmd_vel -> STOP khẩn cấp
  if (_last_cmd_ms == 0 || (current_ms - _last_cmd_ms > _cmd_timeout_ms)) {
    _target_rpm_left = 0.0f;
    _target_rpm_right = 0.0f;
    _status = DRIVE_STATUS_TIMEOUT;
    return;
  }

  // 3. Kiểm tra an toàn: Lệnh ngắt v=0, w=0 -> STOP
  if (fabsf(_last_linear_x) < 0.001f && fabsf(_last_angular_z) < 0.001f) {
    _target_rpm_left = 0.0f;
    _target_rpm_right = 0.0f;
    _status = DRIVE_STATUS_STOPPED;
    return;
  }

  // 4. Differential Drive Inverse Kinematics
  float v = constrain(_last_linear_x, -_max_linear_mps, _max_linear_mps);
  float w = constrain(_last_angular_z, -_max_angular_radps, _max_angular_radps);

  float v_left = v - (w * _wheel_separation_m / 2.0f);
  float v_right = v + (w * _wheel_separation_m / 2.0f);

  float circumference = M_PI * _wheel_diameter_m;
  float rpm_left_raw = (v_left / circumference) * 60.0f;
  float rpm_right_raw = (v_right / circumference) * 60.0f;

  // 5. Clamping & Scaling tỷ lệ RPM nếu vượt quá MAX_WHEEL_RPM
  float max_mag = fmaxf(fabsf(rpm_left_raw), fabsf(rpm_right_raw));
  if (max_mag > _max_wheel_rpm) {
    float scale = _max_wheel_rpm / max_mag;
    _target_rpm_left = rpm_left_raw * scale;
    _target_rpm_right = rpm_right_raw * scale;
    _status = DRIVE_STATUS_CLAMPED;
  } else {
    _target_rpm_left = rpm_left_raw;
    _target_rpm_right = rpm_right_raw;
    _status = DRIVE_STATUS_OK;
  }

  // 6. Policy vùng tốc độ thấp (Low-speed Dead-zone Policy)
  // Giữ nguyên Target RPM thực tế được yêu cầu, KHÔNG cưỡng bức lên 40 RPM.
  // Chỉ triệt tiêu các lệnh quá nhỏ dưới ngưỡng _min_actuation_rpm (8.0 RPM) để tránh nhiễu encoder 38 PPR.
  
  // Bánh Trái
  float mag_left = fabsf(_target_rpm_left);
  if (mag_left > 0.0f && mag_left < _min_actuation_rpm) {
    _target_rpm_left = 0.0f;
  }

  // Bánh Phải
  float mag_right = fabsf(_target_rpm_right);
  if (mag_right > 0.0f && mag_right < _min_actuation_rpm) {
    _target_rpm_right = 0.0f;
  }
}

const char* DifferentialDrive::getStatusString() const {
  switch (_status) {
    case DRIVE_STATUS_OK:           return "CMD_OK";
    case DRIVE_STATUS_TIMEOUT:      return "CMD_TIMEOUT";
    case DRIVE_STATUS_DISCONNECTED: return "AGENT_DISCONNECTED";
    case DRIVE_STATUS_STOPPED:      return "STOPPED";
    case DRIVE_STATUS_CLAMPED:      return "RPM_CLAMPED";
    default:                        return "UNKNOWN";
  }
}
