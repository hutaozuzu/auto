#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <micro_ros_utilities/type_utilities.h>

// ==========================================
// CẤU HÌNH PHẦN CỨNG ESP32 #2 (DEDICATED LIDAR)
// ==========================================
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define LIDAR_BAUDRATE 115200

#define LIDAR_MCTR_PIN 18
#define LIDAR_PWM_CHANNEL 0
#define LIDAR_PWM_FREQ 10000
#define LIDAR_PWM_RESOLUTION 8
#define LIDAR_PWM_DUTY 128  // ~50% duty cycle for initial target ~4.5 - 5.0 Hz physical scan rate

HardwareSerial LidarSerial(2);

const uint8_t PH1 = 0xAA;
const uint8_t PH2 = 0x55;
const uint8_t PH3 = 0x66;
const uint16_t TRI_PACKHEADSIZE = 10;
const uint16_t MAX_NODES = 80;

uint16_t checksumErrorCount = 0;

enum ParserState {
  STATE_WAIT_AA,
  STATE_WAIT_55,
  STATE_READ_HEADER,
  STATE_READ_DATA,
  STATE_SKIP_TIMESTAMP
};

ParserState currentState = STATE_WAIT_AA;
uint8_t packetBuffer[10 + MAX_NODES * 2];
uint16_t writeIndex = 0;
uint8_t expectedDataLength = 0;
uint8_t nodeCount = 0;
uint8_t skipCounter = 0;

// DIRECT BINNING & DOUBLE BUFFERS
#define NUM_BINS 720
#define ANGLE_INCREMENT_DEG (360.0f / (float)NUM_BINS)

struct LaserScanBuffer {
  float ranges[NUM_BINS];
  uint16_t valid_point_count;
  int32_t stamp_sec;
  uint32_t stamp_nanosec;
  float scan_time;
  float time_increment;
  uint32_t seq_id;
};

static LaserScanBuffer scan_buffers[2];
static volatile int active_acquire_idx = 0;
static volatile int completed_publish_idx = -1;
static portMUX_TYPE buffer_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t publisher_task_handle = NULL;

// Diagnostic Counters
static volatile uint32_t cnt_uart_bytes = 0;
static volatile uint32_t cnt_decoded_packets = 0;
static volatile uint32_t cnt_valid_points = 0;
static volatile uint32_t cnt_raw_wraps = 0;
static volatile uint32_t cnt_complete_scans = 0;
static volatile uint32_t cnt_scan_handoffs = 0;
static volatile uint32_t cnt_scan_overwrites = 0;
static volatile uint32_t cnt_publish_calls = 0;
static volatile uint32_t cnt_publish_ok = 0;
static volatile uint32_t cnt_publish_fail = 0;

sensor_msgs__msg__LaserScan scan_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t scan_publisher;

bool micro_ros_connected = false;
uint32_t publish_error_count = 0;
uint32_t last_reconnect_attempt = 0;

static uint32_t previous_wrap_us = 0;
static bool firstScanCompleted = false;

// Forward Declarations
void parsePacket(const uint8_t* packet, uint16_t length);
void completeScan();
void resetActiveScan();

// Checksum Verification
bool checkChecksum(const uint8_t* packet, uint16_t length) {
  if (length % 2 != 0) return false;
  uint16_t computed = 0;
  for (uint16_t i = 0; i < length; i += 2) {
    if (i == 8) continue;
    uint16_t word = packet[i] | (packet[i + 1] << 8);
    computed ^= word;
  }
  uint16_t received = packet[8] | (packet[9] << 8);
  return computed == received;
}

void resetActiveScan() {
  LaserScanBuffer* buf = &scan_buffers[active_acquire_idx];
  for (int i = 0; i < NUM_BINS; i++) {
    buf->ranges[i] = INFINITY;
  }
  buf->valid_point_count = 0;
}

static uint32_t first_ray_local_us = 0;
static uint32_t last_ray_local_us = 0;
static int64_t first_ray_epoch_ns = 0;
static uint32_t previous_first_ray_local_us = 0;

void processPoint(float angle_deg, uint16_t distance_mm) {
  if (angle_deg < 0.0f || angle_deg >= 360.0f) return;
  
  float dist_m = (float)distance_mm / 1000.0f;
  if (dist_m < 0.11f || dist_m > 8.0f) return;

  int bin_idx = (int)floor(angle_deg / ANGLE_INCREMENT_DEG);
  if (bin_idx < 0) bin_idx = 0;
  if (bin_idx >= NUM_BINS) bin_idx = NUM_BINS - 1;

  LaserScanBuffer* buf = &scan_buffers[active_acquire_idx];
  if (!isfinite(buf->ranges[bin_idx]) || dist_m < buf->ranges[bin_idx]) {
    if (!isfinite(buf->ranges[bin_idx])) {
      if (buf->valid_point_count == 0) {
        first_ray_local_us = micros();
        if (rmw_uros_epoch_synchronized()) {
          first_ray_epoch_ns = rmw_uros_epoch_nanos();
        } else {
          first_ray_epoch_ns = (int64_t)millis() * 1000000LL;
        }
      }
      last_ray_local_us = micros();
      buf->valid_point_count++;
      cnt_valid_points++;
    }
    buf->ranges[bin_idx] = dist_m;
  }
}

void completeScan() {
  cnt_raw_wraps++;
  uint32_t current_wrap_us = micros();

  if (!firstScanCompleted) {
    firstScanCompleted = true;
    previous_first_ray_local_us = first_ray_local_us;
    previous_wrap_us = current_wrap_us;
    resetActiveScan();
    return;
  }

  if (scan_buffers[active_acquire_idx].valid_point_count < 50) {
    resetActiveScan();
    return;
  }

  cnt_complete_scans++;

  // 1. scan_time = Full revolution period from FIRST RAY(scan N) to FIRST RAY(scan N+1)
  uint32_t scan_period_us = 0;
  if (previous_first_ray_local_us > 0 && first_ray_local_us > previous_first_ray_local_us) {
    scan_period_us = first_ray_local_us - previous_first_ray_local_us;
  } else {
    scan_period_us = current_wrap_us - previous_wrap_us;
  }
  previous_first_ray_local_us = first_ray_local_us;
  previous_wrap_us = current_wrap_us;

  // 2. active_acquisition_duration = Time from FIRST RAY to LAST RAY within current scan
  uint32_t active_dur_us = 0;
  if (last_ray_local_us >= first_ray_local_us && first_ray_local_us > 0) {
    active_dur_us = last_ray_local_us - first_ray_local_us;
  }
  if (active_dur_us == 0) {
    active_dur_us = scan_period_us;
  }

  float scan_time = (float)scan_period_us / 1000000.0f;
  float time_increment = ((float)active_dur_us / 1000000.0f) / (float)(NUM_BINS - 1);

  scan_buffers[active_acquire_idx].scan_time = scan_time;
  scan_buffers[active_acquire_idx].time_increment = time_increment;

  // ROS 2 sensor_msgs/LaserScan semantics: header.stamp must represent FIRST RAY acquisition time
  scan_buffers[active_acquire_idx].stamp_sec = (int32_t)(first_ray_epoch_ns / 1000000000LL);
  scan_buffers[active_acquire_idx].stamp_nanosec = (uint32_t)(first_ray_epoch_ns % 1000000000LL);
  scan_buffers[active_acquire_idx].seq_id = cnt_complete_scans;

  // Handoff to publisher using Double Buffer with critical section (Non-blocking)
  taskENTER_CRITICAL(&buffer_mux);
  if (completed_publish_idx != -1) {
    cnt_scan_overwrites++; // Overload policy: latest scan wins
  }
  completed_publish_idx = active_acquire_idx;
  active_acquire_idx = 1 - active_acquire_idx;
  cnt_scan_handoffs++;
  taskEXIT_CRITICAL(&buffer_mux);

  // Immediately prepare the new active buffer so UART reading is NEVER paused!
  resetActiveScan();

  // Notify publisher task on Core 0
  if (publisher_task_handle != NULL) {
    vTaskNotifyGiveFromISR(publisher_task_handle, NULL);
  }
}

void parsePacket(const uint8_t* packet, uint16_t length) {
  cnt_decoded_packets++;
  uint8_t ct = packet[2];
  uint8_t count = packet[3];
  uint16_t firstAngleRaw = packet[4] | (packet[5] << 8);
  uint16_t lastAngleRaw = packet[6] | (packet[7] << 8);
  
  if (!(firstAngleRaw & 0x01) || !(lastAngleRaw & 0x01)) return;
  
  double sa = (firstAngleRaw >> 1) / 64.0;
  double ea = (lastAngleRaw >> 1) / 64.0;
  if (ea < sa) ea += 360.0;
  
  double step = 0.0;
  if (count > 1) step = (ea - sa) / (count - 1);
  
  static float previousAngle = -1.0f;
  static bool hasPreviousAngle = false;
  
  for (uint8_t i = 0; i < count; i++) {
    uint16_t d_raw = packet[10 + 2 * i] | (packet[10 + 2 * i + 1] << 8);
    if (d_raw > 0) {
      double distance_mm = d_raw / 4.0;
      if (distance_mm >= 110.0 && distance_mm <= 8000.0) {
        double angle_initial = sa + step * i;
        double ca = atan(17.8 / distance_mm) * 180.0 / PI;
        double angle_final = angle_initial + ca;
        if (angle_final >= 360.0) angle_final -= 360.0;
        
        if (hasPreviousAngle) {
          if (previousAngle > 300.0f && angle_final < 60.0f) {
            completeScan();
          }
        }
        previousAngle = angle_final;
        hasPreviousAngle = true;

        // Convert hardware Clockwise angle to standard ROS Counter-Clockwise angle
        double angle_ros = 360.0 - angle_final;
        while (angle_ros >= 360.0) angle_ros -= 360.0;
        while (angle_ros < 0.0) angle_ros += 360.0;

        processPoint(angle_ros, (uint16_t)distance_mm);
      }
    }
  }
}

void processIncomingByte(uint8_t c) {
  cnt_uart_bytes++;
  switch (currentState) {
    case STATE_WAIT_AA:
      if (c == PH1) { packetBuffer[0] = c; writeIndex = 1; currentState = STATE_WAIT_55; }
      break;
    case STATE_WAIT_55:
      if (c == PH2) { packetBuffer[1] = c; writeIndex = 2; currentState = STATE_READ_HEADER; }
      else if (c == PH3) { skipCounter = 6; currentState = STATE_SKIP_TIMESTAMP; }
      else if (c == PH1) { packetBuffer[0] = PH1; writeIndex = 1; }
      else { currentState = STATE_WAIT_AA; }
      break;
    case STATE_READ_HEADER:
      packetBuffer[writeIndex++] = c;
      if (writeIndex == TRI_PACKHEADSIZE) {
        nodeCount = packetBuffer[3];
        if (nodeCount > MAX_NODES) { currentState = STATE_WAIT_AA; }
        else {
          expectedDataLength = nodeCount * 2;
          if (expectedDataLength > 0) { currentState = STATE_READ_DATA; }
          else {
            if (checkChecksum(packetBuffer, TRI_PACKHEADSIZE)) parsePacket(packetBuffer, TRI_PACKHEADSIZE);
            currentState = STATE_WAIT_AA;
          }
        }
      }
      break;
    case STATE_READ_DATA:
      packetBuffer[writeIndex++] = c;
      if (writeIndex == TRI_PACKHEADSIZE + expectedDataLength) {
        if (checkChecksum(packetBuffer, writeIndex)) parsePacket(packetBuffer, writeIndex);
        currentState = STATE_WAIT_AA;
      }
      break;
    case STATE_SKIP_TIMESTAMP:
      skipCounter--;
      if (skipCounter == 0) currentState = STATE_WAIT_AA;
      break;
  }
}

void initLaserScanMessage() {
  static micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 20;
  conf.max_ros2_type_sequence_capacity = NUM_BINS;
  conf.max_basic_type_sequence_capacity = NUM_BINS;

  bool success = micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    &scan_msg,
    conf
  );

  if (success) {
    strcpy(scan_msg.header.frame_id.data, "laser_frame");
    scan_msg.header.frame_id.size = strlen("laser_frame");
    
    scan_msg.angle_min = 0.0f;
    scan_msg.angle_max = 2.0f * M_PI - (2.0f * M_PI / (float)NUM_BINS);
    scan_msg.angle_increment = 2.0f * M_PI / (float)NUM_BINS;
    scan_msg.range_min = 0.11f;
    scan_msg.range_max = 8.0f;

    scan_msg.ranges.size = NUM_BINS;
    scan_msg.intensities.size = NUM_BINS;

    for (int i = 0; i < NUM_BINS; i++) {
      scan_msg.ranges.data[i] = INFINITY;
      scan_msg.intensities.data[i] = 0.0f;
    }
  } else {
    Serial.println("[ERROR] Không thể khởi tạo bộ nhớ cho /scan LaserScan message!");
  }
}

bool initMicroROS() {
  IPAddress agent_ip(192, 168, 1, 23);
  size_t agent_port = 8888;
  char ssid[] = "KHONGBIET";
  char password[] = "tram2009";

  Serial.printf("Đang kết nối Wi-Fi: %s ...\n", ssid);
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  uint32_t wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[ERROR] Kết nối Wi-Fi thất bại!");
    return false;
  }

  Serial.printf("\nWi-Fi đã kết nối! IP ESP32 #2: %s\n", WiFi.localIP().toString().c_str());

  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
  if (rcl_init_options_set_domain_id(&init_options, 0) != RCL_RET_OK) return false;

  if (rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "s2_lidar_node", "", &support) != RCL_RET_OK) return false;

  initLaserScanMessage();

  if (rclc_publisher_init_default(
    &scan_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "/scan"
  ) != RCL_RET_OK) return false;

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;

  rmw_uros_sync_session(1000);

  Serial.println("[INFO] ESP32 #2 (Dedicated LiDAR) khởi tạo micro-ROS thành công!");
  return true;
}

// FreeRTOS Task B: micro-ROS Publisher (Runs on Core 0, Priority 1)
void publisherTask(void* pvParameters) {
  while (1) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));

    int idx_to_pub = -1;
    taskENTER_CRITICAL(&buffer_mux);
    idx_to_pub = completed_publish_idx;
    completed_publish_idx = -1;
    taskEXIT_CRITICAL(&buffer_mux);

    if (idx_to_pub != -1 && micro_ros_connected) {
      LaserScanBuffer* buf = &scan_buffers[idx_to_pub];

      scan_msg.header.stamp.sec = buf->stamp_sec;
      scan_msg.header.stamp.nanosec = buf->stamp_nanosec;
      scan_msg.scan_time = buf->scan_time;
      scan_msg.time_increment = buf->time_increment;

      memcpy(scan_msg.ranges.data, buf->ranges, NUM_BINS * sizeof(float));

      cnt_publish_calls++;
      rcl_ret_t rc = rcl_publish(&scan_publisher, &scan_msg, NULL);
      if (rc == RCL_RET_OK) {
        cnt_publish_ok++;
        publish_error_count = 0;
      } else {
        cnt_publish_fail++;
        publish_error_count++;
        if (publish_error_count > 20) {
          micro_ros_connected = false;
        }
      }
    }

    if (micro_ros_connected) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    } else {
      uint32_t now = millis();
      if (now - last_reconnect_attempt > 5000) {
        last_reconnect_attempt = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
          if (initMicroROS()) {
            micro_ros_connected = true;
            publish_error_count = 0;
            Serial.println("[INFO] Kết nối lại micro-ROS Agent thành công!");
          }
        }
      }
    }
  }
}

// FreeRTOS Task A: Dedicated UART Acquisition (Runs on Core 1, Priority 5)
void uartAcquisitionTask(void* pvParameters) {
  while (1) {
    while (LidarSerial.available() > 0) {
      uint8_t c = LidarSerial.read();
      processIncomingByte(c);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n===== ESP32 #2 DEDICATED YDLIDAR S2 NON-BLOCKING PIPELINE (PLAN C) =====");

  // Khởi tạo phần cứng PWM trên GPIO18 điều khiển tốc độ động cơ LiDAR S2
  ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RESOLUTION);
  ledcAttachPin(LIDAR_MCTR_PIN, LIDAR_PWM_CHANNEL);
  ledcWrite(LIDAR_PWM_CHANNEL, LIDAR_PWM_DUTY);
  Serial.printf("[INFO] Đã khởi tạo PWM GPIO%d (Duty: %d/255, Freq: %d Hz)\n", LIDAR_MCTR_PIN, LIDAR_PWM_DUTY, LIDAR_PWM_FREQ);

  // Mở rộng bộ đệm phần mềm UART lên 2048 bytes để đảm bảo an toàn tuyệt đối
  LidarSerial.setRxBufferSize(2048);
  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  // Khởi tạo 2 bộ đệm Double Buffer
  for (int b = 0; b < 2; b++) {
    for (int i = 0; i < NUM_BINS; i++) {
      scan_buffers[b].ranges[i] = INFINITY;
    }
    scan_buffers[b].valid_point_count = 0;
  }

  if (initMicroROS()) {
    micro_ros_connected = true;
    publish_error_count = 0;
  } else {
    Serial.println("[ERROR] Khởi tạo micro-ROS thất bại. Sẽ tự động kết nối lại.");
    micro_ros_connected = false;
  }

  // Khởi tạo FreeRTOS Dual Tasks:
  // Task B: Publisher trên Core 0 (Priority 1)
  xTaskCreatePinnedToCore(publisherTask, "pub_task", 6144, NULL, 1, &publisher_task_handle, 0);

  // Task A: UART Acquisition trên Core 1 (Priority 5)
  xTaskCreatePinnedToCore(uartAcquisitionTask, "uart_task", 4096, NULL, 5, NULL, 1);

  Serial.println("[INFO] FreeRTOS Non-Blocking Double-Buffer Pipeline đã khởi tạo thành công!");
}

void loop() {
  // Loop task rảnh rỗi hoặc có thể in telemetry định kỳ 5s
  static uint32_t lastDiagMs = 0;
  uint32_t now = millis();
  if (now - lastDiagMs > 5000) {
    lastDiagMs = now;
    Serial.printf("[DIAG] UART Bytes: %u | Pkts: %u | Pts: %u | Wraps: %u | Scans: %u | Handoffs: %u | Overwrites: %u | PubOK: %u\n",
      cnt_uart_bytes, cnt_decoded_packets, cnt_valid_points, cnt_raw_wraps, cnt_complete_scans, cnt_scan_handoffs, cnt_scan_overwrites, cnt_publish_ok);
  }
  delay(100);
}
