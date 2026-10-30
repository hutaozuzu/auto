#include "odometry_calibration.h"
#include "speed_control_test.h"
#include "wheel_odometry.h"
#include <Preferences.h>

// Biến tích lũy ngắt đếm xung
static volatile uint32_t left_delta_pulses = 0;
static volatile uint32_t right_delta_pulses = 0;

static volatile uint64_t left_total_pulses = 0;
static volatile uint64_t right_total_pulses = 0;

static volatile uint32_t last_left_isr_us = 0;
static volatile uint32_t last_right_isr_us = 0;

void IRAM_ATTR leftOdomISR() {
  uint32_t now_us = micros();
  // Lọc nhiễu ngắt Debounce Filter 1000us (chống Crosstalk)
  if (now_us - last_left_isr_us >= 1000) {
    left_delta_pulses++;
    left_total_pulses++;
    last_left_isr_us = now_us;
  }
}

void IRAM_ATTR rightOdomISR() {
  uint32_t now_us = micros();
  // Lọc nhiễu ngắt Debounce Filter 1000us (chống Crosstalk)
  if (now_us - last_right_isr_us >= 1000) {
    right_delta_pulses++;
    right_total_pulses++;
    last_right_isr_us = now_us;
  }
}

void setupOdometryInterrupts() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftOdomISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightOdomISR, RISING);
}

// Atomic Snapshot lấy số xung delta chu kỳ
static uint32_t fetchAndResetLeftDelta() {
  noInterrupts();
  uint32_t count = left_delta_pulses;
  left_delta_pulses = 0;
  interrupts();
  return count;
}

static uint32_t fetchAndResetRightDelta() {
  noInterrupts();
  uint32_t count = right_delta_pulses;
  right_delta_pulses = 0;
  interrupts();
  return count;
}

// Atomic Snapshot lấy tổng số xung tích lũy (Total Pulses)
static void getTotals(uint64_t* l_tot, uint64_t* r_tot) {
  noInterrupts();
  *l_tot = left_total_pulses;
  *r_tot = right_total_pulses;
  interrupts();
}

static void resetTotals() {
  noInterrupts();
  left_delta_pulses = 0;
  right_delta_pulses = 0;
  left_total_pulses = 0;
  right_total_pulses = 0;
  interrupts();
}

static Preferences prefs;
static WheelOdometry wheelOdom(DEFAULT_WHEEL_DIAMETER_M, DEFAULT_WHEEL_SEPARATION_M, PULSES_PER_REV);

static const char* getTestModeName(Phase3BTestMode mode) {
  switch (mode) {
    case TEST_MODE_STRAIGHT:    return "STRAIGHT";
    case TEST_MODE_ROTATE_LEFT: return "ROTATE_LEFT";
    case TEST_MODE_ROTATE_RIGHT:return "ROTATE_RIGHT";
    default:                    return "UNKNOWN";
  }
}

static void saveResultToNVS(uint32_t test_mode, uint64_t fin_l_p, uint64_t fin_r_p, float fin_l_d, float fin_r_d, float x_m, float y_m, float th_tot_rad, float th_tot_deg, float th_norm_rad, float th_norm_deg) {
  prefs.begin("odom_pose", false);
  prefs.putUInt("valid", 1);
  prefs.putUInt("test_mode", test_mode);
  prefs.putULong64("fin_l_p", fin_l_p);
  prefs.putULong64("fin_r_p", fin_r_p);
  prefs.putFloat("fin_l_d", fin_l_d);
  prefs.putFloat("fin_r_d", fin_r_d);
  prefs.putFloat("x_m", x_m);
  prefs.putFloat("y_m", y_m);
  prefs.putFloat("th_tot_rad", th_tot_rad);
  prefs.putFloat("th_tot_deg", th_tot_deg);
  prefs.putFloat("th_norm_rad", th_norm_rad);
  prefs.putFloat("th_norm_deg", th_norm_deg);
  prefs.end();
}

void runOdometryCalibration(MotorDriver& motors, SpeedController& leftCtrl, SpeedController& rightCtrl) {
  static CalibTestState state = CALIB_INIT;
  static uint32_t lastControlMs = 0;
  static uint32_t stateStartMs = 0;
  static uint32_t runStartMs = 0;
  static uint32_t coastStartMs = 0;
  static uint32_t lastReportMs = 0;

  static float left_rpm_history[3] = {0.0f, 0.0f, 0.0f};
  static float right_rpm_history[3] = {0.0f, 0.0f, 0.0f};
  static uint8_t history_idx = 0;

  static uint16_t zero_pulse_sample_count = 0;

  // CHỐT KẾT QUẢ POSE CUỐI CÙNG
  static uint64_t final_l_tot = 0;
  static uint64_t final_r_tot = 0;
  static float final_l_dist = 0.0f;
  static float final_r_dist = 0.0f;
  static float final_x = 0.0f;
  static float final_y = 0.0f;

  // 2 BIẾN GÓC TÁCH BIỆT: TOTAL HOÀN TOÀN VÀ NORMALIZED [-PI, +PI]
  static float final_theta_tot_rad = 0.0f;
  static float final_theta_tot_deg = 0.0f;
  static float final_theta_norm_rad = 0.0f;
  static float final_theta_norm_deg = 0.0f;

  const Phase3BTestMode test_mode = ACTIVE_PHASE3B_TEST_MODE;

  uint32_t now = millis();

  // 1. TRẠNG THÁI KHỞI TẠO (CHẠY KHI BOOT HOẶC KHI CẮM LẠI USB RESET ESP32)
  if (state == CALIB_INIT) {
    motors.stop();
    leftCtrl.reset();
    rightCtrl.reset();
    wheelOdom.reset();
    for (int i = 0; i < 3; i++) { left_rpm_history[i] = 0.0f; right_rpm_history[i] = 0.0f; }

    Serial.println("\n==========================================================");
    Serial.println("[PHASE 3B.2] VERIFY TOTAL ROTATION BEFORE ROS2 /ODOM");
    Serial.println("==========================================================");
    Serial.printf("TEST_MODE          : %s\n", getTestModeName(test_mode));
    Serial.printf("PULSES_PER_REV     : %d\n", PULSES_PER_REV);
    Serial.printf("WHEEL_DIAMETER_M   : %.4f m (%d mm)\n", DEFAULT_WHEEL_DIAMETER_M, (int)(DEFAULT_WHEEL_DIAMETER_M * 1000));
    Serial.printf("WHEEL_SEPARATION_M : %.4f m (%d mm)\n", DEFAULT_WHEEL_SEPARATION_M, (int)(DEFAULT_WHEEL_SEPARATION_M * 1000));
    Serial.printf("ODOM_UPDATE_PERIOD : %d ms\n", ODOM_UPDATE_PERIOD_MS);

    // KIỂM TRA DỮ LIỆU ĐÃ LƯU TRONG FLASH NVS
    prefs.begin("odom_pose", true);
    uint32_t valid = prefs.getUInt("valid", 0);
    if (valid == 1) {
      uint32_t saved_mode = prefs.getUInt("test_mode", 0);
      final_l_tot = prefs.getULong64("fin_l_p", 0);
      final_r_tot = prefs.getULong64("fin_r_p", 0);
      final_l_dist = prefs.getFloat("fin_l_d", 0.0f);
      final_r_dist = prefs.getFloat("fin_r_d", 0.0f);
      final_x = prefs.getFloat("x_m", 0.0f);
      final_y = prefs.getFloat("y_m", 0.0f);
      final_theta_tot_rad = prefs.getFloat("th_tot_rad", 0.0f);
      final_theta_tot_deg = prefs.getFloat("th_tot_deg", 0.0f);
      final_theta_norm_rad = prefs.getFloat("th_norm_rad", 0.0f);
      final_theta_norm_deg = prefs.getFloat("th_norm_deg", 0.0f);
      prefs.end();

      Serial.println("\n===== PHASE 3B.2 ROTATION RESULT =====");
      Serial.printf("MODE=%s\n", getTestModeName((Phase3BTestMode)saved_mode));
      Serial.printf("FINAL_LEFT_PULSES=%llu\n", final_l_tot);
      Serial.printf("FINAL_RIGHT_PULSES=%llu\n", final_r_tot);
      Serial.printf("FINAL_LEFT_DISTANCE_M=%.4f\n", final_l_dist);
      Serial.printf("FINAL_RIGHT_DISTANCE_M=%.4f\n", final_r_dist);
      Serial.printf("FINAL_X_M=%.4f\n", final_x);
      Serial.printf("FINAL_Y_M=%.4f\n", final_y);
      Serial.printf("THETA_TOTAL_RAD=%.4f\n", final_theta_tot_rad);
      Serial.printf("THETA_TOTAL_DEG=%.2f\n", final_theta_tot_deg);
      Serial.printf("THETA_NORMALIZED_RAD=%.4f\n", final_theta_norm_rad);
      Serial.printf("THETA_NORMALIZED_DEG=%.2f\n", final_theta_norm_deg);
      Serial.println("STATUS=COMPLETE");
      Serial.println("=====================================\n");
    } else {
      prefs.end();
    }

    Serial.println("----------------------------------------------------------");
    Serial.println(">>> WAITING 5 SECONDS COUNTDOWN... HÃY ĐẶT ROBOT VÀO VẠCH XUẤT PHÁT! <<<\n");

    state = CALIB_WAITING_COUNTDOWN;
    stateStartMs = now;
    lastControlMs = now;
    return;
  }

  // 2. TRẠNG THÁI PHỤC VỤ TEST COMPLETE (IN FINAL RESULT ĐỊNH KỲ 1 GIÂY)
  if (state == CALIB_TEST_COMPLETE) {
    motors.stop();
    if (now - lastReportMs >= 1000) {
      lastReportMs = now;
      Serial.println("\n===== PHASE 3B.2 ROTATION RESULT =====");
      Serial.printf("MODE=%s\n", getTestModeName(test_mode));
      Serial.printf("FINAL_LEFT_PULSES=%llu\n", final_l_tot);
      Serial.printf("FINAL_RIGHT_PULSES=%llu\n", final_r_tot);
      Serial.printf("FINAL_LEFT_DISTANCE_M=%.4f\n", final_l_dist);
      Serial.printf("FINAL_RIGHT_DISTANCE_M=%.4f\n", final_r_dist);
      Serial.printf("FINAL_X_M=%.4f\n", final_x);
      Serial.printf("FINAL_Y_M=%.4f\n", final_y);
      Serial.printf("THETA_TOTAL_RAD=%.4f\n", final_theta_tot_rad);
      Serial.printf("THETA_TOTAL_DEG=%.2f\n", final_theta_tot_deg);
      Serial.printf("THETA_NORMALIZED_RAD=%.4f\n", final_theta_norm_rad);
      Serial.printf("THETA_NORMALIZED_DEG=%.2f\n", final_theta_norm_deg);
      Serial.println("STATUS=COMPLETE");
      Serial.println("=================================");
    }
    return;
  }

  // A. TRẠNG THÁI WAITING COUNTDOWN (5 GIÂY NGHỈ ĐỨNG YÊN TẠI VẠCH)
  if (state == CALIB_WAITING_COUNTDOWN) {
    motors.stop();
    uint32_t waitElapsed = now - stateStartMs;
    static uint32_t lastSecondPrint = 0;

    if (waitElapsed - lastSecondPrint >= 1000) {
      lastSecondPrint = waitElapsed;
      uint32_t remainingSec = (COUNTDOWN_WAIT_MS > waitElapsed) ? ((COUNTDOWN_WAIT_MS - waitElapsed) / 1000 + 1) : 0;
      Serial.printf("[COUNTDOWN] Robot starting in %lu seconds...\n", remainingSec);
    }

    if (waitElapsed >= COUNTDOWN_WAIT_MS) {
      resetTotals();
      leftCtrl.reset();
      rightCtrl.reset();
      wheelOdom.reset();

      state = CALIB_POSE_RUNNING;
      runStartMs = now;
      Serial.println("\n----------------------------------------------------------");
      Serial.printf(">>> [POSE_RUNNING] EXECUTING TEST_MODE: %s (DURATION = 3.0s)... <<<\n", getTestModeName(test_mode));
      Serial.println("l_raw,r_raw,l_sign,r_sign,l_signed,r_signed,delta_l_m,delta_r_m,delta_s_m,delta_theta_rad,x_m,y_m,th_tot_deg,th_norm_deg");
      Serial.println("----------------------------------------------------------");
    }
    return;
  }

  // 3. VÒNG LẶP CẬP NHẬT POSE & ĐIỀU KHIỂN CHU KỲ ODOM_UPDATE_PERIOD_MS (100MS = 10HZ)
  if (now - lastControlMs < ODOM_UPDATE_PERIOD_MS) {
    return;
  }

  uint32_t elapsed_ms = now - lastControlMs;
  float dt_seconds = (float)elapsed_ms / 1000.0f;
  lastControlMs = now;

  // Static Direction Memory (Duy trì dấu chiều cho giai đoạn COASTING đến khi dừng hẳn)
  static int8_t last_nonzero_left_dir = 1;
  static int8_t last_nonzero_right_dir = 1;

  int8_t left_dir_sign = 0;
  int8_t right_dir_sign = 0;

  // Xác định chiều động cơ từ Motor Command hiện tại hoặc duy trì từ Last Non-Zero Direction
  if (state == CALIB_POSE_RUNNING) {
    if (test_mode == TEST_MODE_STRAIGHT) {
      left_dir_sign = 1;   // Forward (+)
      right_dir_sign = 1;  // Forward (+)
    } else if (test_mode == TEST_MODE_ROTATE_LEFT) {
      left_dir_sign = -1;  // Reverse (-) cho Bánh Trái Lùi
      right_dir_sign = 1;  // Forward (+) cho Bánh Phải Tiến
    } else if (test_mode == TEST_MODE_ROTATE_RIGHT) {
      left_dir_sign = 1;   // Forward (+) cho Bánh Trái Tiến
      right_dir_sign = -1; // Reverse (-) cho Bánh Phải Lùi
    }

    if (left_dir_sign != 0)  last_nonzero_left_dir = left_dir_sign;
    if (right_dir_sign != 0) last_nonzero_right_dir = right_dir_sign;
  } else if (state == CALIB_COASTING) {
    // Khi COASTING (PWM = 0), duy trì dấu chiều trước khi ngắt PWM
    left_dir_sign = last_nonzero_left_dir;
    right_dir_sign = last_nonzero_right_dir;
  }

  // Atomic snapshot delta magnitude từ FC-03 ISR (Luôn >= 0)
  uint32_t l_delta_raw = fetchAndResetLeftDelta();
  uint32_t r_delta_raw = fetchAndResetRightDelta();

  // Signed Delta Pulses Layer (Chuyển sang int32_t an toàn)
  int32_t l_delta_signed = (int32_t)l_delta_raw * left_dir_sign;
  int32_t r_delta_signed = (int32_t)r_delta_raw * right_dir_sign;

  // Tích phân Midpoint Pose Integration từ Signed Delta Pulses
  float delta_s_m = 0.0f, delta_theta_rad = 0.0f;
  wheelOdom.update(l_delta_signed, r_delta_signed, &delta_s_m, &delta_theta_rad);

  float rpm_raw_left = ((float)l_delta_raw / (float)PULSES_PER_REV) * (60000.0f / (float)elapsed_ms);
  float rpm_raw_right = ((float)r_delta_raw / (float)PULSES_PER_REV) * (60000.0f / (float)elapsed_ms);

  if (rpm_raw_left <= MAX_PHYSICAL_RPM_LIMIT) left_rpm_history[history_idx] = rpm_raw_left;
  if (rpm_raw_right <= MAX_PHYSICAL_RPM_LIMIT) right_rpm_history[history_idx] = rpm_raw_right;
  history_idx = (history_idx + 1) % 3;

  float rpm_filtered_left = (left_rpm_history[0] + left_rpm_history[1] + left_rpm_history[2]) / 3.0f;
  float rpm_filtered_right = (right_rpm_history[0] + right_rpm_history[1] + right_rpm_history[2]) / 3.0f;

  // B. TRẠNG THÁI POSE RUNNING (CHẠY 3.0s THEO TEST MODE)
  if (state == CALIB_POSE_RUNNING) {
    uint32_t runElapsed = now - runStartMs;

    // Thiết lập Target RPM/PWM theo Test Mode
    if (test_mode == TEST_MODE_STRAIGHT) {
      leftCtrl.setTargetRPM(CALIB_TARGET_RPM);
      rightCtrl.setTargetRPM(CALIB_TARGET_RPM);

      float lp = 0, li = 0, rp = 0, ri = 0;
      uint8_t lff = 0, rff = 0;
      uint8_t lpwm = leftCtrl.update(rpm_filtered_left, dt_seconds, &lp, &li, &lff);
      uint8_t rpwm = rightCtrl.update(rpm_filtered_right, dt_seconds, &rp, &ri, &rff);
      motors.setSpeed(lpwm, rpwm);
    } 
    else if (test_mode == TEST_MODE_ROTATE_LEFT) {
      motors.setSpeed(140, -140);
    } 
    else if (test_mode == TEST_MODE_ROTATE_RIGHT) {
      motors.setSpeed(-140, 140);
    }

    float delta_l_m = (float)l_delta_signed * wheelOdom.getDistancePerPulse();
    float delta_r_m = (float)r_delta_signed * wheelOdom.getDistancePerPulse();

    // Log CSV chuẩn 14 cột hiển thị cả th_tot_deg và th_norm_deg
    Serial.printf("%u,%u,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n",
                  l_delta_raw, r_delta_raw,
                  left_dir_sign, right_dir_sign,
                  l_delta_signed, r_delta_signed,
                  delta_l_m, delta_r_m,
                  delta_s_m, delta_theta_rad,
                  wheelOdom.getX(), wheelOdom.getY(),
                  wheelOdom.getThetaDegTotal(), wheelOdom.getThetaDeg());

    // KHI ĐẠT ĐỦ 3000 MS: NGẮT PWM = 0 HOÀN TOÀN VÀ CHUYỂN SANG COASTING
    if (runElapsed >= EXACT_RUN_DURATION_MS) {
      motors.stop();
      leftCtrl.setTargetRPM(0.0f);
      rightCtrl.setTargetRPM(0.0f);

      state = CALIB_COASTING;
      coastStartMs = now;
      zero_pulse_sample_count = 0;
      Serial.println("\n----------------------------------------------------------");
      Serial.println(">>> [COASTING] PWM = 0. CONTINUING POSE INTEGRATION UNTIL FULL STOP... <<<");
      Serial.println("----------------------------------------------------------");
    }
    return;
  }

  // C. TRẠNG THÁI COASTING (PWM = 0, TIẾP TỤC TÍCH PHÂN POSE TRÔI QUÁN TÍNH CHỜ BÁNH DỪNG HẲN)
  if (state == CALIB_COASTING) {
    motors.stop(); // Khóa PWM = 0 tuyệt đối

    uint32_t coastElapsed = now - coastStartMs;

    if (l_delta_raw == 0 && r_delta_raw == 0) {
      zero_pulse_sample_count++;
    } else {
      zero_pulse_sample_count = 0;
    }

    bool full_stop_detected = (zero_pulse_sample_count >= STOP_CONFIRM_SAMPLES);
    bool timeout_reached = (coastElapsed >= COASTING_TIMEOUT_MS);

    float delta_l_m = (float)l_delta_signed * wheelOdom.getDistancePerPulse();
    float delta_r_m = (float)r_delta_signed * wheelOdom.getDistancePerPulse();

    Serial.printf("%u,%u,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n",
                  l_delta_raw, r_delta_raw,
                  left_dir_sign, right_dir_sign,
                  l_delta_signed, r_delta_signed,
                  delta_l_m, delta_r_m,
                  delta_s_m, delta_theta_rad,
                  wheelOdom.getX(), wheelOdom.getY(),
                  wheelOdom.getThetaDegTotal(), wheelOdom.getThetaDeg());

    if (full_stop_detected || timeout_reached) {
      motors.stop();

      getTotals(&final_l_tot, &final_r_tot);
      final_l_dist = (float)final_l_tot * wheelOdom.getDistancePerPulse();
      final_r_dist = (float)final_r_tot * wheelOdom.getDistancePerPulse();

      final_x = wheelOdom.getX();
      final_y = wheelOdom.getY();
      final_theta_tot_rad = wheelOdom.getThetaRadTotal();
      final_theta_tot_deg = wheelOdom.getThetaDegTotal();
      final_theta_norm_rad = wheelOdom.getThetaRad();
      final_theta_norm_deg = wheelOdom.getThetaDeg();

      // LƯU TOÀN BỘ KẾT QUẢ VÀO FLASH NVS
      saveResultToNVS((uint32_t)test_mode, final_l_tot, final_r_tot, final_l_dist, final_r_dist, final_x, final_y, final_theta_tot_rad, final_theta_tot_deg, final_theta_norm_rad, final_theta_norm_deg);

      state = CALIB_TEST_COMPLETE;
      lastReportMs = 0;
      if (timeout_reached && !full_stop_detected) {
        Serial.println("\n[WARNING] COASTING TIMEOUT REACHED BEFORE FULL STOP DETECTED!");
      } else {
        Serial.println("\n[SUCCESS] FULL STOP DETECTED CLEANLY!");
      }
    }
  }
}
