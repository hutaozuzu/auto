#include "encoder_test.h"

// ==========================================
// CÁC BIẾN TOÀN CỤC ISR (VOLATILE TỐI ƯU)
// ==========================================
static volatile uint32_t left_encoder_count = 0;
static volatile uint32_t right_encoder_count = 0;

// Ngắt ISR cho Encoder Trái (Tối giản tối đa)
void IRAM_ATTR leftEncoderISR() {
  left_encoder_count++;
}

// Ngắt ISR cho Encoder Phải (Tối giản tối đa)
void IRAM_ATTR rightEncoderISR() {
  right_encoder_count++;
}

void setupEncoderInterrupts() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT);

  // Gán ngắt RISING khi khe đĩa che/bật ánh sáng ngắt hồng ngoại
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), rightEncoderISR, RISING);
}

// Đọc giá trị ngắt an toàn (Atomic read)
static uint32_t getLeftEncoderCount() {
  noInterrupts();
  uint32_t count = left_encoder_count;
  interrupts();
  return count;
}

static uint32_t getRightEncoderCount() {
  noInterrupts();
  uint32_t count = right_encoder_count;
  interrupts();
  return count;
}

static void resetEncoderCounts() {
  noInterrupts();
  left_encoder_count = 0;
  right_encoder_count = 0;
  interrupts();
}

void runStandaloneEncoderMotorTest(MotorDriver& motors, uint32_t target_left_pulses, uint32_t target_right_pulses) {
  static TestMode currentMode = MODE_READY_LEFT;
  static uint32_t lastPrintMs = 0;
  static bool headerPrinted = false;

  uint32_t now = millis();
  uint32_t currentLeft = getLeftEncoderCount();
  uint32_t currentRight = getRightEncoderCount();

  // Đọc phím bấm từ Serial Monitor
  char inputChar = 0;
  if (Serial.available() > 0) {
    inputChar = Serial.read();
  }

  switch (currentMode) {
    // ---------------------------------------------------------
    // BƯỚC 1: CHỜ PHÍM 'n' ĐỂ BẬT MOTOR TRÁI
    // ---------------------------------------------------------
    case MODE_READY_LEFT: {
      if (!headerPrinted) {
        motors.stop();
        resetEncoderCounts();
        Serial.println("\n==========================================================");
        Serial.println("[PHASE 2A] CHƯƠNG TRÌNH TEST INTERACTIVE PHÍM 'n'");
        Serial.println("==========================================================");
        Serial.println("-> HƯỚNG DẪN:");
        Serial.println("   1. Gửi phím 'n' LẦN 1 -> Motor TRÁI bắt đầu quay.");
        Serial.println("   2. Quan sát bánh quay đủ 1 vòng, gửi phím 'n' LẦN 2 -> Motor TRÁI DỪNG NGAY.");
        Serial.println("   3. Gửi phím 'n' LẦN 3 -> Motor PHẢI bắt đầu quay.");
        Serial.println("   4. Quan sát bánh quay đủ 1 vòng, gửi phím 'n' LẦN 4 -> Motor PHẢI DỪNG NGAY.");
        Serial.println("----------------------------------------------------------");
        Serial.println("[BƯỚC 1] Nhập phím 'n' để CHO MOTOR TRÁI CHẠY...");
        headerPrinted = true;
      }

      if (inputChar == 'n' || inputChar == 'N') {
        resetEncoderCounts();
        motors.setLeftMotor(DEFAULT_TEST_PWM);
        Serial.println("\n[MOTOR TRÁI ĐÃ BẬT] Đang quay (PWM 110)... Gửi phím 'n' để DỪNG khi đủ 1 vòng!");
        currentMode = MODE_RUNNING_LEFT;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 2: MOTOR TRÁI ĐANG QUAY -> TỰ DỪNG HOẶC BẤM 'n' ĐỂ DỪNG
    // ---------------------------------------------------------
    case MODE_RUNNING_LEFT: {
      if (now - lastPrintMs >= 150) {
        lastPrintMs = now;
        Serial.printf("[RUNNING LEFT] Pulses Count: %u | PWM: %d (Gửi 'n' để dừng)\n", currentLeft, DEFAULT_TEST_PWM);
      }

      // Dừng khi gõ 'n' HOẶC tự dừng khi đạt target_left_pulses
      if (inputChar == 'n' || inputChar == 'N' || currentLeft >= target_left_pulses) {
        motors.stop();
        Serial.println("----------------------------------------------------------");
        Serial.printf("[MOTOR TRÁI ĐÃ DỪNG] Số xung đếm được thực tế: %u pulses\n", currentLeft);
        Serial.println("----------------------------------------------------------");
        Serial.println("[BƯỚC 2] Nhập phím 'n' để CHO MOTOR PHẢI CHẠY...");
        currentMode = MODE_READY_RIGHT;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 3: CHỜ PHÍM 'n' ĐỂ BẬT MOTOR PHẢI
    // ---------------------------------------------------------
    case MODE_READY_RIGHT: {
      if (inputChar == 'n' || inputChar == 'N') {
        resetEncoderCounts();
        motors.setRightMotor(DEFAULT_TEST_PWM);
        Serial.println("\n[MOTOR PHẢI ĐÃ BẬT] Đang quay (PWM 110)... Gửi phím 'n' để DỪNG khi đủ 1 vòng!");
        currentMode = MODE_RUNNING_RIGHT;
      }
      break;
    }

    // ---------------------------------------------------------
    // BƯỚC 4: MOTOR PHẢI ĐANG QUAY -> TỰ DỪNG HOẶC BẤM 'n' ĐỂ DỪNG
    // ---------------------------------------------------------
    case MODE_RUNNING_RIGHT: {
      if (now - lastPrintMs >= 150) {
        lastPrintMs = now;
        Serial.printf("[RUNNING RIGHT] Pulses Count: %u | PWM: %d (Gửi 'n' để dừng)\n", currentRight, DEFAULT_TEST_PWM);
      }

      // Dừng khi gõ 'n' HOẶC tự dừng khi đạt target_right_pulses
      if (inputChar == 'n' || inputChar == 'N' || currentRight >= target_right_pulses) {
        motors.stop();
        Serial.println("----------------------------------------------------------");
        Serial.printf("[MOTOR PHẢI ĐÃ DỪNG] Số xung đếm được thực tế: %u pulses\n", currentRight);
        Serial.println("==========================================================");
        Serial.println("[COMPLETE] BÀI TEST THỦ CÔNG TỪNG BÊN HOÀN THÀNH TỐT!");
        Serial.println("==========================================================");
        currentMode = MODE_COMPLETED;
      }
      break;
    }

    case MODE_COMPLETED:
      motors.stop();
      break;
  }
}
