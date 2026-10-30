#include "speed_control_test.h"

// Biến ngắt và thời điểm ngắt gần nhất để chống nhiễu sóng điện từ (EMI Debounce Filter)
static volatile uint32_t left_speed_pulses = 0;
static volatile uint32_t right_speed_pulses = 0;
static volatile uint32_t last_left_isr_us = 0;
static volatile uint32_t last_right_isr_us = 0;

void IRAM_ATTR leftControlISR() {
  uint32_t now_us = micros();
  // Lọc nhiễu xung cao tần cách nhau dưới 1000 microsecond (chống Crosstalk từ motor)
  if (now_us - last_left_isr_us >= 1000) {
    left_speed_pulses++;
    last_left_isr_us = now_us;
  }
}

void IRAM_ATTR rightControlISR() {
  uint32_t now_us = micros();
  // Lọc nhiễu xung cao tần cách nhau dưới 1000 microsecond (chống Crosstalk từ motor)
  if (now_us - last_right_isr_us >= 1000) {
    right_speed_pulses++;
    last_right_isr_us = now_us;
  }
}

void setupSpeedControlInterrupts() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftControlISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightControlISR, RISING);
}

static uint32_t fetchAndResetLeftPulses() {
  noInterrupts();
  uint32_t count = left_speed_pulses;
  left_speed_pulses = 0;
  interrupts();
  return count;
}

static uint32_t fetchAndResetRightPulses() {
  noInterrupts();
  uint32_t count = right_speed_pulses;
  right_speed_pulses = 0;
  interrupts();
  return count;
}

void runClosedLoopSpeedTest(MotorDriver& motors, SpeedController& leftCtrl, SpeedController& rightCtrl) {
  static ControlTestState state = CTRL_INIT;
  static uint32_t lastControlMs = 0;
  static uint32_t stateStartMs = 0;

  static uint32_t lastLeftPulseChangeMs = 0;
  static uint32_t lastRightPulseChangeMs = 0;

  static float left_rpm_history[3] = {0.0f, 0.0f, 0.0f};
  static float right_rpm_history[3] = {0.0f, 0.0f, 0.0f};
  static uint8_t history_idx = 0;

  uint32_t now = millis();

  if (now - lastControlMs < CONTROL_PERIOD_MS) {
    return;
  }

  uint32_t elapsed_ms = now - lastControlMs;
  float dt_seconds = (float)elapsed_ms / 1000.0f;
  lastControlMs = now;

  uint32_t left_pulses = fetchAndResetLeftPulses();
  uint32_t right_pulses = fetchAndResetRightPulses();

  float rpm_raw_left = ((float)left_pulses / (float)PULSES_PER_REV) * (60000.0f / (float)elapsed_ms);
  float rpm_raw_right = ((float)right_pulses / (float)PULSES_PER_REV) * (60000.0f / (float)elapsed_ms);

  bool left_sample_valid = true;
  bool right_sample_valid = true;

  if (rpm_raw_left > MAX_PHYSICAL_RPM_LIMIT) {
    left_sample_valid = false;
    Serial.printf("[INVALID RPM SAMPLE][LEFT] elapsed_ms=%u, pulses=%u, rpm_raw=%.2f\n", elapsed_ms, left_pulses, rpm_raw_left);
  }
  if (rpm_raw_right > MAX_PHYSICAL_RPM_LIMIT) {
    right_sample_valid = false;
    Serial.printf("[INVALID RPM SAMPLE][RIGHT] elapsed_ms=%u, pulses=%u, rpm_raw=%.2f\n", elapsed_ms, right_pulses, rpm_raw_right);
  }

  if (left_sample_valid) left_rpm_history[history_idx] = rpm_raw_left;
  if (right_sample_valid) right_rpm_history[history_idx] = rpm_raw_right;
  history_idx = (history_idx + 1) % 3;

  float rpm_filtered_left = (left_rpm_history[0] + left_rpm_history[1] + left_rpm_history[2]) / 3.0f;
  float rpm_filtered_right = (right_rpm_history[0] + right_rpm_history[1] + right_rpm_history[2]) / 3.0f;

  if (left_pulses > 0) lastLeftPulseChangeMs = now;
  if (right_pulses > 0) lastRightPulseChangeMs = now;

  switch (state) {
    case CTRL_INIT: {
      motors.stop();
      leftCtrl.reset();
      rightCtrl.reset();
      fetchAndResetLeftPulses();
      fetchAndResetRightPulses();
      for (int i = 0; i < 3; i++) { left_rpm_history[i] = 0.0f; right_rpm_history[i] = 0.0f; }

      Serial.println("\n==========================================================");
      Serial.println("[PHASE 2C.4] FINAL VERIFICATION BOTH 80 RPM (15 GIÂY)");
      Serial.println("==========================================================");
      Serial.println("time_ms,phase,side,target_rpm,elapsed_ms,pulse_count,rpm_raw,rpm_filtered,error,pwm_ff,p_term,i_term,pwm_output,status");
      Serial.println("----------------------------------------------------------");
      Serial.println(">>> [VERIFICATION] BOTH 80 RPM (15 GIÂY)... <<<\n");

      leftCtrl.setGains(INITIAL_KP_LEFT, INITIAL_KI_LEFT);
      rightCtrl.setGains(INITIAL_KP_RIGHT, INITIAL_KI_RIGHT);
      leftCtrl.setTargetRPM(80.0f);
      rightCtrl.setTargetRPM(80.0f);

      state = CTRL_SWEEP_RIGHT_OPEN_LOOP;
      stateStartMs = now;
      lastLeftPulseChangeMs = now;
      lastRightPulseChangeMs = now;
      break;
    }

    case CTRL_SWEEP_RIGHT_OPEN_LOOP: {
      float lp = 0, li = 0, rp = 0, ri = 0;
      uint8_t lff = 0, rff = 0;
      uint8_t lpwm = leftCtrl.update(rpm_filtered_left, dt_seconds, &lp, &li, &lff);
      uint8_t rpwm = rightCtrl.update(rpm_filtered_right, dt_seconds, &rp, &ri, &rff);

      motors.setSpeed(lpwm, rpwm);

      Serial.printf("%lu,BOTH_80,L,80.00,%u,%u,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%u,%s\n",
                    now, elapsed_ms, left_pulses, rpm_raw_left, rpm_filtered_left, 80.0f - rpm_filtered_left, lff, lp, li, lpwm, left_sample_valid ? "OK" : "INVALID_SAMPLE");
      Serial.printf("%lu,BOTH_80,R,80.00,%u,%u,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%u,%s\n",
                    now, elapsed_ms, right_pulses, rpm_raw_right, rpm_filtered_right, 80.0f - rpm_filtered_right, rff, rp, ri, rpwm, right_sample_valid ? "OK" : "INVALID_SAMPLE");

      if (now - stateStartMs >= 15000) {
        motors.stop();
        leftCtrl.setTargetRPM(0.0f);
        rightCtrl.setTargetRPM(0.0f);
        leftCtrl.reset();
        rightCtrl.reset();
        Serial.println("==========================================================");
        Serial.println("[PHASE 2C.4 COMPLETE] HOÀN THÀNH TEST VERIFICATION BOTH 80 RPM!");
        Serial.println("==========================================================");
        state = CTRL_DONE;
      }
      break;
    }

    case CTRL_DONE:
      motors.stop();
      break;
  }
}
