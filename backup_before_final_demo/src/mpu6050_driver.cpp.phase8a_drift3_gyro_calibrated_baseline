#include "mpu6050_driver.h"

#define MPU6050_REG_DEVID        0x75
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

MPU6050Driver::MPU6050Driver(int sda_pin, int scl_pin)
    : _i2c_addr(0x68),
      _sda_pin(sda_pin),
      _scl_pin(scl_pin),
      _gyro_bias_x(0.0f),
      _gyro_bias_y(0.0f),
      _gyro_bias_z(0.0f),
      _gyro_yaw_total_rad(0.0f),
      _last_imu_sample_us(0),
      _is_initialized(false),
      _is_calibrated(false) {}

bool MPU6050Driver::begin(uint32_t i2c_freq) {
  Wire.begin(_sda_pin, _scl_pin);
  Wire.setClock(100000); // 100 kHz I2C Standard mode chuẩn an toàn cho MPU6050

  // Probe I2C address (thử 0x68 trước, sau đó 0x69)
  uint8_t addrs[2] = {0x68, 0x69};
  bool found = false;

  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(addrs[i]);
    if (Wire.endTransmission() == 0) {
      _i2c_addr = addrs[i];
      found = true;
      break;
    }
  }

  if (!found) {
    Serial.printf("[ERROR] MPU6050 không phản hồi trên bus I2C (SDA=%d, SCL=%d)!\n", _sda_pin, _scl_pin);
    _is_initialized = false;
    return false;
  }

  Serial.printf("[INFO] Đã tìm thấy cảm biến MPU6050 tại địa chỉ I2C 0x%02X!\n", _i2c_addr);

  // 1. Wake Up MPU6050 (Ghi 0x00 vào PWR_MGMT_1)
  Wire.beginTransmission(_i2c_addr);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    _is_initialized = false;
    return false;
  }
  delay(10);

  // 2. Cấu hình Dải đo Gyro ±250 deg/s (0x00)
  Wire.beginTransmission(_i2c_addr);
  Wire.write(MPU6050_REG_GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  // 3. Cấu hình Dải đo Accel ±2g (0x00)
  Wire.beginTransmission(_i2c_addr);
  Wire.write(MPU6050_REG_ACCEL_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  _is_initialized = true;
  return true;
}

GyroCalibResult MPU6050Driver::calibrateGyro(uint16_t num_samples, uint32_t settling_ms, bool (*is_moving_check)()) {
  GyroCalibResult result = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, false};
  if (!_is_initialized) return result;

  Serial.printf("[INFO] Bắt đầu quá trình khởi động MPU6050: Chờ ổn định cảm biến (%u ms)...\n", settling_ms);
  delay(settling_ms);

  Serial.printf("[INFO] Đang thu thập %u mẫu Gyro (~%.1f s, giữ robot đứng yên tuyệt đối)...\n",
                num_samples, (float)(num_samples * 5) / 1000.0f);

  uint32_t start_ms = millis();
  float sum_gx = 0.0f;
  float sum_gy = 0.0f;
  float sum_gz = 0.0f;
  float sum_sq_gz = 0.0f;
  uint16_t valid_count = 0;
  bool motion_detected = false;

  const float deg_to_rad = M_PI / 180.0f;

  for (uint16_t i = 0; i < num_samples; i++) {
    // Kiểm tra an toàn chuyển động từ encoder (Motion Rejection)
    if (is_moving_check && is_moving_check()) {
      motion_detected = true;
    }

    Wire.beginTransmission(_i2c_addr);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(_i2c_addr, (uint8_t)14);
      if (Wire.available() >= 14) {
        Wire.read(); Wire.read(); // ax
        Wire.read(); Wire.read(); // ay
        Wire.read(); Wire.read(); // az
        Wire.read(); Wire.read(); // temp

        int16_t raw_gx = (Wire.read() << 8) | Wire.read();
        int16_t raw_gy = (Wire.read() << 8) | Wire.read();
        int16_t raw_gz = (Wire.read() << 8) | Wire.read();

        float gx_rad = ((float)raw_gx / RAW_GYRO_SCALE) * deg_to_rad;
        float gy_rad = ((float)raw_gy / RAW_GYRO_SCALE) * deg_to_rad;
        float gz_rad = ((float)raw_gz / RAW_GYRO_SCALE) * deg_to_rad;

        sum_gx += gx_rad;
        sum_gy += gy_rad;
        sum_gz += gz_rad;
        sum_sq_gz += (gz_rad * gz_rad);
        valid_count++;
      }
    }
    delay(5); // ~5ms mỗi mẫu (tổng ~5s cho 1000 mẫu)
  }

  uint32_t duration_ms = millis() - start_ms;
  result.samples_collected = valid_count;
  result.duration_sec = (float)duration_ms / 1000.0f;
  result.motion_detected = motion_detected;

  if (valid_count > 0) {
    float mean_gx = sum_gx / (float)valid_count;
    float mean_gy = sum_gy / (float)valid_count;
    float mean_gz = sum_gz / (float)valid_count;

    float variance_gz = (sum_sq_gz / (float)valid_count) - (mean_gz * mean_gz);
    float std_gz = (variance_gz > 0.0f) ? sqrtf(variance_gz) : 0.0f;

    result.bias_x = mean_gx;
    result.bias_y = mean_gy;
    result.bias_z = mean_gz;
    result.std_z = std_gz;

    // Ngưỡng kiểm tra an toàn: nếu encoder quay hoặc dao động gyro quá lớn (> 0.05 rad/s ~ 2.8 deg/s)
    if (motion_detected || std_gz > 0.05f) {
      result.success = false;
      Serial.printf("[WARNING] Hiệu chuẩn thất bại do phát hiện chuyển động! (Motion: %s, StdZ: %.5f rad/s)\n",
                    motion_detected ? "YES" : "NO", std_gz);
    } else {
      _gyro_bias_x = mean_gx;
      _gyro_bias_y = mean_gy;
      _gyro_bias_z = mean_gz;
      _gyro_std_z = std_gz;
      _is_calibrated = true;
      result.success = true;

      Serial.println("==================================================");
      Serial.println("[INFO] KẾT QUẢ HIỆU CHUẨN GYRO BIAS STARTUP:");
      Serial.printf("  Số mẫu thu thập  : %u mẫu\n", valid_count);
      Serial.printf("  Thời gian đo     : %.2f s (Settling: %.2f s)\n", result.duration_sec, (float)settling_ms / 1000.0f);
      Serial.printf("  Gyro-Z Bias      : %.6f rad/s (%+.4f deg/s)\n", _gyro_bias_z, _gyro_bias_z * 180.0f / M_PI);
      Serial.printf("  Gyro-Z Std Dev   : %.6f rad/s (%.4f deg/s)\n", _gyro_std_z, _gyro_std_z * 180.0f / M_PI);
      Serial.printf("  Phát hiện rung lắc: %s\n", motion_detected ? "YES" : "NO");
      Serial.println("  Trạng thái       : THÀNH CÔNG (SUCCESS)");
      Serial.println("==================================================");
    }
  }

  return result;
}

bool MPU6050Driver::readIMU(IMUData &data) {
  if (!_is_initialized) return false;

  Wire.beginTransmission(_i2c_addr);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  if (Wire.endTransmission() != 0) return false;

  Wire.requestFrom(_i2c_addr, (uint8_t)14);
  if (Wire.available() < 14) return false;

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  const float deg_to_rad = M_PI / 180.0f;

  // Chuyển đổi Accel sang m/s^2
  data.accel_x = ((float)raw_ax / RAW_ACCEL_SCALE) * G_TO_MS2;
  data.accel_y = ((float)raw_ay / RAW_ACCEL_SCALE) * G_TO_MS2;
  data.accel_z = ((float)raw_az / RAW_ACCEL_SCALE) * G_TO_MS2;

  // Chuyển đổi Gyro sang rad/s và trừ đi Gyro Bias
  float gyro_x_raw_radps = ((float)raw_gx / RAW_GYRO_SCALE) * deg_to_rad;
  float gyro_y_raw_radps = ((float)raw_gy / RAW_GYRO_SCALE) * deg_to_rad;
  float gyro_z_raw_radps = ((float)raw_gz / RAW_GYRO_SCALE) * deg_to_rad;

  data.gyro_x = gyro_x_raw_radps - _gyro_bias_x;
  data.gyro_y = gyro_y_raw_radps - _gyro_bias_y;
  data.gyro_z = gyro_z_raw_radps - _gyro_bias_z;

  return true;
}
