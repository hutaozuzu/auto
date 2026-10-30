#include "motor_speed_test.h"

// ==========================================
// BIẾN TOÀN CỤC ISR (VOLATILE TỐI ƯU)
// ==========================================
static volatile uint32_t left_pulse_count = 0;
static volatile uint32_t right_pulse_count = 0;

void IRAM_ATTR leftSpeedISR() {
  left_pulse_count++;
}

void IRAM_ATTR rightSpeedISR() {
  right_pulse_count++;
}

void setupSpeedTestInterrupts() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftSpeedISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightSpeedISR, RISING);
}

static uint32_t fetchAndResetLeftPulseCount() {
  noInterrupts();
  uint32_t count = left_pulse_count;
  left_pulse_count = 0;
  interrupts();
  return count;
}

static uint32_t fetchAndResetRightPulseCount() {
  noInterrupts();
  uint32_t count = right_pulse_count;
  right_pulse_count = 0;
  interrupts();
  return count;
}

static void clearAllSpeedPulses() {
  noInterrupts();
  left_pulse_count = 0;
  right_pulse_count = 0;
  interrupts();
}

void runMotorSpeedCharacterization(MotorDriver& motors) {
  static SpeedTestState state = SPEED_TEST_INIT;
  static uint8_t currentStepIdx = 0;
  static uint32_t stateStartMs = 0;
  static uint32_t lastPulseCountCheck = 0;
  static uint32_t lastPulseChangeMs = 0;

  uint32_t now = millis();

  switch (state) {
    // ---------------------------------------------------------
    // BƯỚC 1: KHỞI TẠO BÀI TEST TỐC ĐỘ
    // ---------------------------------------------------------
    case SPEED_TEST_INIT: {
      motors.stop();
      clearAllSpeedPulses();
      Serial.println("\n==========================================================");
      Serial.println("[PHASE 2B] CHƯƠNG TRÌNH ĐO ĐẶC TÍNH TỐC ĐỘ MOTOR (PWM -> RPM)");
      Serial.println("==========================================================");
      Serial.println("-> Cửa sổ đo (Window): 1000 ms");
      Serial.println("-> Thời gian Warm-up: 2000 ms | Thời gian nghỉ: 1000 ms");
      Serial.println("-> Công thức: RPM = pulse_count * 60.0 / 38.0");
      Serial.println("----------------------------------------------------------");
      Serial.println(">>> BẮT ĐẦU TEST MOTOR TRÁI (LEFT MOTOR)... <<<");
      
      currentStepIdx = 0;
      state = SPEED_TEST_LEFT_WARMUP;
      stateStartMs = now;
      lastPulseChangeMs = now;
      lastPulseCountCheck = 0;
      motors.setLeftMotor(PWM_TEST_POINTS[currentStepIdx]);
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 2: MOTOR TRÁI WARM-UP 2 GIÂY
    // ---------------------------------------------------------
    case SPEED_TEST_LEFT_WARMUP: {
      if (now - stateStartMs >= WARMUP_TIME_MS) {
        // Hết thời gian warm-up 2s -> Reset xung và bắt đầu đo chính xác trong 1s
        clearAllSpeedPulses();
        state = SPEED_TEST_LEFT_MEASURE;
        stateStartMs = now;
        lastPulseChangeMs = now;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 3: ĐO RPM MOTOR TRÁI TRONG 1000 MS
    // ---------------------------------------------------------
    case SPEED_TEST_LEFT_MEASURE: {
      uint32_t pulses = fetchAndResetLeftPulseCount();
      // Gom dồn pulse trong 1s
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

      // Cảnh báo Stall nếu trong 1.5s không nhận được xung nào
      if (now - lastPulseChangeMs > STALL_CHECK_TIMEOUT_MS && left_window_pulses == 0) {
        Serial.printf("[MOTOR LEFT] PWM = %d | ENCODER NO PULSE / MOTOR STALL DETECTED!\n", PWM_TEST_POINTS[currentStepIdx]);
      }

      if (now - measureStartMs >= MEASURE_WINDOW_MS) {
        float rpm = ((float)left_window_pulses * 60.0f) / (float)PULSES_PER_REV;
        
        Serial.println("[MOTOR LEFT]");
        Serial.printf("PWM = %d\n", PWM_TEST_POINTS[currentStepIdx]);
        Serial.println("Window = 1000 ms");
        Serial.printf("Pulse count = %u\n", left_window_pulses);
        Serial.printf("RPM = %.2f\n\n", rpm);

        motors.stop();
        measureStartMs = 0;
        state = SPEED_TEST_LEFT_REST;
        stateStartMs = now;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 4: MOTOR TRÁI NGHỈ 1 GIÂY GIỮA CÁC MỨC PWM
    // ---------------------------------------------------------
    case SPEED_TEST_LEFT_REST: {
      if (now - stateStartMs >= REST_TIME_MS) {
        currentStepIdx++;
        if (currentStepIdx < NUM_PWM_STEPS) {
          state = SPEED_TEST_LEFT_WARMUP;
          stateStartMs = now;
          lastPulseChangeMs = now;
          motors.setLeftMotor(PWM_TEST_POINTS[currentStepIdx]);
        } else {
          Serial.println("----------------------------------------------------------");
          Serial.println(">>> HOÀN THÀNH TEST MOTOR TRÁI! TẮT 2 GIÂY TRƯỚC KHI SANG MOTOR PHẢI... <<<");
          Serial.println("----------------------------------------------------------");
          motors.stop();
          currentStepIdx = 0;
          state = SPEED_TEST_RIGHT_WARMUP;
          stateStartMs = now + 1000; // Thêm 1s nghỉ
        }
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 5: MOTOR PHẢI WARM-UP 2 GIÂY
    // ---------------------------------------------------------
    case SPEED_TEST_RIGHT_WARMUP: {
      if (now - stateStartMs >= WARMUP_TIME_MS) {
        clearAllSpeedPulses();
        state = SPEED_TEST_RIGHT_MEASURE;
        stateStartMs = now;
        lastPulseChangeMs = now;
      } else {
        motors.setRightMotor(PWM_TEST_POINTS[currentStepIdx]);
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 6: ĐO RPM MOTOR PHẢI TRONG 1000 MS
    // ---------------------------------------------------------
    case SPEED_TEST_RIGHT_MEASURE: {
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
        Serial.printf("[MOTOR RIGHT] PWM = %d | ENCODER NO PULSE / MOTOR STALL DETECTED!\n", PWM_TEST_POINTS[currentStepIdx]);
      }

      if (now - measureRightStartMs >= MEASURE_WINDOW_MS) {
        float rpm = ((float)right_window_pulses * 60.0f) / (float)PULSES_PER_REV;

        Serial.println("[MOTOR RIGHT]");
        Serial.printf("PWM = %d\n", PWM_TEST_POINTS[currentStepIdx]);
        Serial.println("Window = 1000 ms");
        Serial.printf("Pulse count = %u\n", right_window_pulses);
        Serial.printf("RPM = %.2f\n\n", rpm);

        motors.stop();
        measureRightStartMs = 0;
        state = SPEED_TEST_RIGHT_REST;
        stateStartMs = now;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 7: MOTOR PHẢI NGHỈ 1 GIÂY GIỮA CÁC MỨC PWM
    // ---------------------------------------------------------
    case SPEED_TEST_RIGHT_REST: {
      if (now - stateStartMs >= REST_TIME_MS) {
        currentStepIdx++;
        if (currentStepIdx < NUM_PWM_STEPS) {
          state = SPEED_TEST_RIGHT_WARMUP;
          stateStartMs = now;
          lastPulseChangeMs = now;
          motors.setRightMotor(PWM_TEST_POINTS[currentStepIdx]);
        } else {
          motors.stop();
          Serial.println("==========================================================");
          Serial.println("[PHASE 2B COMPLETE] HOÀN THÀNH TOÀN BỘ ĐO ĐẶC TÍNH PWM -> RPM!");
          Serial.println("==========================================================");
          state = SPEED_TEST_DONE;
        }
      }
      break;
    }

    case SPEED_TEST_DONE:
      motors.stop();
      break;
  }
}
