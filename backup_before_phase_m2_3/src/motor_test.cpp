#include "motor_test.h"

enum MotorTestState {
  TEST_INIT,
  TEST_FORWARD_2S,
  TEST_STOP_1,
  TEST_BACKWARD_2S,
  TEST_STOP_2,
  TEST_TURN_LEFT_2S,
  TEST_STOP_3,
  TEST_TURN_RIGHT_2S,
  TEST_STOP_4,
  TEST_SLOW_FORWARD_2S,
  TEST_FAST_FORWARD_2S,
  TEST_DONE
};

static MotorTestState testState = TEST_INIT;
static uint32_t lastStateChangeMs = 0;

void runNonBlockingMotorTest(MotorDriver& motors) {
  uint32_t now = millis();

  switch (testState) {
    case TEST_INIT:
      Serial.println("\n==================================================");
      Serial.println("[MOTOR TEST] BẮT ĐẦU CHƯƠNG TRÌNH TEST MOTOR TB6612FNG");
      Serial.println("==================================================");
      Serial.println("[MOTOR TEST] Bước 1: Tiến 2 giây (Tốc độ êm ái PWM 100)...");
      motors.forward(100);
      lastStateChangeMs = now;
      testState = TEST_FORWARD_2S;
      break;

    case TEST_FORWARD_2S:
      if (now - lastStateChangeMs >= 2000) {
        Serial.println("[MOTOR TEST] Bước 2: Dừng 1 giây...");
        motors.stop();
        lastStateChangeMs = now;
        testState = TEST_STOP_1;
      }
      break;

    case TEST_STOP_1:
      if (now - lastStateChangeMs >= 1000) {
        Serial.println("[MOTOR TEST] Bước 3: Lùi 2 giây (Tốc độ êm ái PWM 100)...");
        motors.backward(100);
        lastStateChangeMs = now;
        testState = TEST_BACKWARD_2S;
      }
      break;

    case TEST_BACKWARD_2S:
      if (now - lastStateChangeMs >= 2000) {
        Serial.println("[MOTOR TEST] Bước 4: Dừng 1 giây...");
        motors.stop();
        lastStateChangeMs = now;
        testState = TEST_STOP_2;
      }
      break;

    case TEST_STOP_2:
      if (now - lastStateChangeMs >= 1000) {
        Serial.println("[MOTOR TEST] Bước 5: Rẽ trái 2 giây (Tốc độ PWM 90)...");
        motors.turnLeft(90);
        lastStateChangeMs = now;
        testState = TEST_TURN_LEFT_2S;
      }
      break;

    case TEST_TURN_LEFT_2S:
      if (now - lastStateChangeMs >= 2000) {
        Serial.println("[MOTOR TEST] Bước 6: Dừng 1 giây...");
        motors.stop();
        lastStateChangeMs = now;
        testState = TEST_STOP_3;
      }
      break;

    case TEST_STOP_3:
      if (now - lastStateChangeMs >= 1000) {
        Serial.println("[MOTOR TEST] Bước 7: Rẽ phải 2 giây (Tốc độ PWM 90)...");
        motors.turnRight(90);
        lastStateChangeMs = now;
        testState = TEST_TURN_RIGHT_2S;
      }
      break;

    case TEST_TURN_RIGHT_2S:
      if (now - lastStateChangeMs >= 2000) {
        Serial.println("[MOTOR TEST] Bước 8: Dừng 1 giây...");
        motors.stop();
        lastStateChangeMs = now;
        testState = TEST_STOP_4;
      }
      break;

    case TEST_STOP_4:
      if (now - lastStateChangeMs >= 1000) {
        Serial.println("[MOTOR TEST] Bước 9: Tiến siêu chậm 2 giây (Tốc độ PWM 60)...");
        motors.forward(60);
        lastStateChangeMs = now;
        testState = TEST_SLOW_FORWARD_2S;
      }
      break;

    case TEST_SLOW_FORWARD_2S:
      if (now - lastStateChangeMs >= 2000) {
        Serial.println("[MOTOR TEST] Bước 10: Tiến vừa 2 giây (Tốc độ PWM 140)...");
        motors.forward(140);
        lastStateChangeMs = now;
        testState = TEST_FAST_FORWARD_2S;
      }
      break;

    case TEST_FAST_FORWARD_2S:
      if (now - lastStateChangeMs >= 2000) {
        Serial.println("==================================================");
        Serial.println("[MOTOR TEST] HOÀN THÀNH TOÀN BỘ KỊCH BẢN TEST! DỪNG MOTOR.");
        Serial.println("==================================================");
        motors.stop();
        testState = TEST_DONE;
      }
      break;

    case TEST_DONE:
      break;
  }
}
