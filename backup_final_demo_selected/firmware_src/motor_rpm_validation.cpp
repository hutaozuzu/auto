#include "motor_rpm_validation.h"

// ==========================================
// BIẾN TOÀN CỤC ISR (VOLATILE TỐI ƯU)
// ==========================================
static volatile uint32_t left_val_pulse_count = 0;
static volatile uint32_t right_val_pulse_count = 0;

void IRAM_ATTR leftValidationISR() {
  left_val_pulse_count++;
}

void IRAM_ATTR rightValidationISR() {
  right_val_pulse_count++;
}

void setupValidationInterrupts() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftValidationISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightValidationISR, RISING);
}

static uint32_t fetchAndResetLeftPulseCount() {
  noInterrupts();
  uint32_t count = left_val_pulse_count;
  left_val_pulse_count = 0;
  interrupts();
  return count;
}

static uint32_t fetchAndResetRightPulseCount() {
  noInterrupts();
  uint32_t count = right_val_pulse_count;
  right_val_pulse_count = 0;
  interrupts();
  return count;
}

static void clearValidationSpeedPulses() {
  noInterrupts();
  left_val_pulse_count = 0;
  right_val_pulse_count = 0;
  interrupts();
}

// Hàm tính toán thống kê (Mean, Min, Max, StdDev) cho mảng 5 lần đo
static void printStatisticalSummary(const char* motor_side, uint8_t pwm, const float rpm_runs[RUNS_PER_PWM]) {
  float sum = 0.0f;
  float min_val = rpm_runs[0];
  float max_val = rpm_runs[0];

  for (int i = 0; i < RUNS_PER_PWM; i++) {
    sum += rpm_runs[i];
    if (rpm_runs[i] < min_val) min_val = rpm_runs[i];
    if (rpm_runs[i] > max_val) max_val = rpm_runs[i];
  }

  float mean = sum / (float)RUNS_PER_PWM;

  float variance_sum = 0.0f;
  for (int i = 0; i < RUNS_PER_PWM; i++) {
    float diff = rpm_runs[i] - mean;
    variance_sum += diff * diff;
  }
  float stddev = sqrt(variance_sum / (float)RUNS_PER_PWM);
  float cv = (mean > 0.001f) ? (stddev / mean * 100.0f) : 0.0f;

  Serial.printf("[%s][PWM=%d][SUMMARY]\n", motor_side, pwm);
  for (int i = 0; i < RUNS_PER_PWM; i++) {
    Serial.printf("rpm_%d=%.2f\n", i + 1, rpm_runs[i]);
  }
  Serial.printf("mean=%.2f\n", mean);
  Serial.printf("min=%.2f\n", min_val);
  Serial.printf("max=%.2f\n", max_val);
  Serial.printf("stddev=%.2f\n", stddev);
  Serial.printf("cv_percent=%.2f%%\n\n", cv);
}

void runMotorRPMValidation(MotorDriver& motors) {
  static ValidationState state = VAL_INIT;
  static uint8_t currentPwmIdx = 0;
  static uint8_t currentRunIdx = 0;
  static uint32_t stateStartMs = 0;
  static uint32_t lastPulseChangeMs = 0;

  static float left_rpm_runs[RUNS_PER_PWM];
  static float right_rpm_runs[RUNS_PER_PWM];

  uint32_t now = millis();

  switch (state) {
    // ---------------------------------------------------------
    // BƯỚC 1: KHỞI TẠO BÀI TEST REPEATED VALIDATION
    // ---------------------------------------------------------
    case VAL_INIT: {
      motors.stop();
      clearValidationSpeedPulses();
      Serial.println("\n==========================================================");
      Serial.println("[PHASE 2B.1] CHƯƠNG TRÌNH XÁC MINH LẶP LẠI RPM (5 RUNS / PWM)");
      Serial.println("==========================================================");
      Serial.printf("-> Cửa sổ đo (Window): %d ms | Warm-up: %d ms | Rest: %d ms\n", 
                    MEASUREMENT_WINDOW_MS, WARMUP_TIME_MS, REST_TIME_MS);
      Serial.println("-> Công thức: RPM = (pulses / 38.0) * (60000.0 / elapsed_ms)");
      Serial.println("----------------------------------------------------------");
      Serial.println(">>> BẮT ĐẦU REPEATED TEST MOTOR TRÁI (LEFT MOTOR)... <<<\n");

      currentPwmIdx = 0;
      currentRunIdx = 0;
      state = VAL_LEFT_WARMUP;
      stateStartMs = now;
      lastPulseChangeMs = now;
      motors.setLeftMotor(VALIDATION_PWM_STEPS[currentPwmIdx]);
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 2: MOTOR TRÁI WARM-UP 2 GIÂY
    // ---------------------------------------------------------
    case VAL_LEFT_WARMUP: {
      if (now - stateStartMs >= WARMUP_TIME_MS) {
        clearValidationSpeedPulses();
        state = VAL_LEFT_MEASURE;
        stateStartMs = now;
        lastPulseChangeMs = now;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 3: ĐO MOTOR TRÁI TRONG 2000 MS (1 LẦN ĐO)
    // ---------------------------------------------------------
    case VAL_LEFT_MEASURE: {
      uint32_t pulses = fetchAndResetLeftPulseCount();
      static uint32_t left_window_pulses = 0;
      static uint32_t measureStartMs = 0;

      if (measureStartMs == 0) {
        measureStartMs = now;
        left_window_pulses = 0;
      }

      left_window_pulses += pulses;

      if (pulses > 0) {
        lastPulseChangeMs = now;
      }

      if (now - lastPulseChangeMs > STALL_CHECK_TIMEOUT_MS && left_window_pulses == 0) {
        Serial.printf("[LEFT][PWM=%d][RUN=%d] MOTOR STALL / ENCODER NO PULSE DETECTED!\n", 
                      VALIDATION_PWM_STEPS[currentPwmIdx], currentRunIdx + 1);
      }

      if (now - measureStartMs >= MEASUREMENT_WINDOW_MS) {
        uint32_t elapsed_ms = now - measureStartMs;
        float rpm = ((float)left_window_pulses / (float)PULSES_PER_REV) * (60000.0f / (float)elapsed_ms);

        left_rpm_runs[currentRunIdx] = rpm;

        Serial.printf("[LEFT][PWM=%d][RUN=%d]\n", VALIDATION_PWM_STEPS[currentPwmIdx], currentRunIdx + 1);
        Serial.printf("elapsed_ms=%u\n", elapsed_ms);
        Serial.printf("pulses=%u\n", left_window_pulses);
        Serial.printf("rpm=%.2f\n\n", rpm);

        motors.stop();
        measureStartMs = 0;
        state = VAL_LEFT_REST;
        stateStartMs = now;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 4: NGHỈ 1 GIÂY GIỮA CÁC LẦN ĐO TRÁI
    // ---------------------------------------------------------
    case VAL_LEFT_REST: {
      if (now - stateStartMs >= REST_TIME_MS) {
        currentRunIdx++;
        if (currentRunIdx < RUNS_PER_PWM) {
          // Tiếp tục lần đo tiếp theo cùng mức PWM
          state = VAL_LEFT_WARMUP;
          stateStartMs = now;
          lastPulseChangeMs = now;
          motors.setLeftMotor(VALIDATION_PWM_STEPS[currentPwmIdx]);
        } else {
          // In bảng thống kê 5 lần đo cho mức PWM hiện tại
          printStatisticalSummary("LEFT", VALIDATION_PWM_STEPS[currentPwmIdx], left_rpm_runs);

          currentRunIdx = 0;
          currentPwmIdx++;
          if (currentPwmIdx < NUM_VALIDATION_STEPS) {
            state = VAL_LEFT_WARMUP;
            stateStartMs = now;
            lastPulseChangeMs = now;
            motors.setLeftMotor(VALIDATION_PWM_STEPS[currentPwmIdx]);
          } else {
            Serial.println("----------------------------------------------------------");
            Serial.println(">>> HOÀN THÀNH TEST MOTOR TRÁI! TẮT 2 GIÂY TRƯỚC KHI SANG MOTOR PHẢI... <<<");
            Serial.println("----------------------------------------------------------\n");
            motors.stop();
            currentPwmIdx = 0;
            currentRunIdx = 0;
            state = VAL_RIGHT_WARMUP;
            stateStartMs = now + 1000;
          }
        }
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 5: MOTOR PHẢI WARM-UP 2 GIÂY
    // ---------------------------------------------------------
    case VAL_RIGHT_WARMUP: {
      if (now - stateStartMs >= WARMUP_TIME_MS) {
        clearValidationSpeedPulses();
        state = VAL_RIGHT_MEASURE;
        stateStartMs = now;
        lastPulseChangeMs = now;
      } else {
        motors.setRightMotor(VALIDATION_PWM_STEPS[currentPwmIdx]);
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 6: ĐO MOTOR PHẢI TRONG 2000 MS (1 LẦN ĐO)
    // ---------------------------------------------------------
    case VAL_RIGHT_MEASURE: {
      uint32_t pulses = fetchAndResetRightPulseCount();
      static uint32_t right_window_pulses = 0;
      static uint32_t measureRightStartMs = 0;

      if (measureRightStartMs == 0) {
        measureRightStartMs = now;
        right_window_pulses = 0;
      }

      right_window_pulses += pulses;

      if (pulses > 0) {
        lastPulseChangeMs = now;
      }

      if (now - lastPulseChangeMs > STALL_CHECK_TIMEOUT_MS && right_window_pulses == 0) {
        Serial.printf("[RIGHT][PWM=%d][RUN=%d] MOTOR STALL / ENCODER NO PULSE DETECTED!\n", 
                      VALIDATION_PWM_STEPS[currentPwmIdx], currentRunIdx + 1);
      }

      if (now - measureRightStartMs >= MEASUREMENT_WINDOW_MS) {
        uint32_t elapsed_ms = now - measureRightStartMs;
        float rpm = ((float)right_window_pulses / (float)PULSES_PER_REV) * (60000.0f / (float)elapsed_ms);

        right_rpm_runs[currentRunIdx] = rpm;

        Serial.printf("[RIGHT][PWM=%d][RUN=%d]\n", VALIDATION_PWM_STEPS[currentPwmIdx], currentRunIdx + 1);
        Serial.printf("elapsed_ms=%u\n", elapsed_ms);
        Serial.printf("pulses=%u\n", right_window_pulses);
        Serial.printf("rpm=%.2f\n\n", rpm);

        motors.stop();
        measureRightStartMs = 0;
        state = VAL_RIGHT_REST;
        stateStartMs = now;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 7: NGHỈ 1 GIÂY GIỮA CÁC LẦN ĐO PHẢI
    // ---------------------------------------------------------
    case VAL_RIGHT_REST: {
      if (now - stateStartMs >= REST_TIME_MS) {
        currentRunIdx++;
        if (currentRunIdx < RUNS_PER_PWM) {
          state = VAL_RIGHT_WARMUP;
          stateStartMs = now;
          lastPulseChangeMs = now;
          motors.setRightMotor(VALIDATION_PWM_STEPS[currentPwmIdx]);
        } else {
          printStatisticalSummary("RIGHT", VALIDATION_PWM_STEPS[currentPwmIdx], right_rpm_runs);

          currentRunIdx = 0;
          currentPwmIdx++;
          if (currentPwmIdx < NUM_VALIDATION_STEPS) {
            state = VAL_RIGHT_WARMUP;
            stateStartMs = now;
            lastPulseChangeMs = now;
            motors.setRightMotor(VALIDATION_PWM_STEPS[currentPwmIdx]);
          } else {
            motors.stop();
            Serial.println("==========================================================");
            Serial.println("[PHASE 2B.1 COMPLETE] HOÀN THÀNH TOÀN BỘ BÀI ĐO LẶP LẠI 5 LẦN!");
            Serial.println("==========================================================");
            state = VAL_DONE;
          }
        }
      }
      break;
    }

    case VAL_DONE:
      motors.stop();
      break;
  }
}
