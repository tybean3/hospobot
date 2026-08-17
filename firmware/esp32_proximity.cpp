#include <Arduino.h>
#include "driver/twai.h"

// ==============================================================================
// Hardware Configurations for ESP32 38-Pin DevKit & TJA1051T/3
// ==============================================================================
#define CAN_TX_PIN GPIO_NUM_4      // TWAI/CAN TX Pin
#define CAN_RX_PIN GPIO_NUM_5      // TWAI/CAN RX Pin

// CAN Node Configuration
#define NODE_ID 0x1A  // ESP32-A (Proximity/Ultrasonics)

// Number of Ultrasonic sensors
#define NUM_SENSORS 10

// Sequential sensor reading scheduling
#define SENSOR_INTERVAL_MS 15  // Time between starting next sensor read (10 * 15ms = 150ms per cycle)
#define HEARTBEAT_INTERVAL_MS 1000

// GPIO assignments for Multiplexer
const uint8_t MUX_S0_PIN = 14;
const uint8_t MUX_S1_PIN = 27;
const uint8_t MUX_S2_PIN = 26;
const uint8_t MUX_S3_PIN = 25;

const uint8_t MUX_TRIG_PIN = 12; // Common TRIG (Mux 1)
const uint8_t MUX_ECHO_PIN = 33; // Common ECHO (Mux 2 - Input only pin)

// Low-Pass Filter Alpha (0.0 to 1.0). Lower = smoother but slower to respond
#define LPF_ALPHA 0.3f

// Buffer to store calculated distances (in meters)
float sensor_distances[NUM_SENSORS] = {-1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f};

// Timing states
unsigned long last_sensor_trigger_time = 0;
unsigned long last_heartbeat_time = 0;
uint8_t current_sensor_index = 0;

// ==============================================================================
// CAN / TWAI Helpers
// ==============================================================================
void initCAN() {
  // 1. Setup TWAI driver configurations
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500kbps CAN Bus Bitrate
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver installed successfully");
  } else {
    Serial.println("Failed to install TWAI Driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver started successfully");
  } else {
    Serial.println("Failed to start TWAI Driver");
  }
}

void sendCANFrame(uint8_t cmd_id, const uint8_t *data, uint8_t dlc) {
  twai_message_t tx_msg;
  tx_msg.identifier = (NODE_ID << 5) | (cmd_id & 0x1F); // 11-bit standard ID
  tx_msg.extd = 0; // Standard format (no extended ID)
  tx_msg.rtr = 0;
  tx_msg.data_length_code = dlc;
  
  if (data != nullptr && dlc > 0) {
    memcpy(tx_msg.data, data, dlc);
  }
  
  // Transmit message with 10ms timeout
  esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
  if (err != ESP_OK) {
    Serial.printf("CAN Transmit failed: 0x%X\n", err);
  }
}

// ==============================================================================
// Multiplexer & Distance Sensor Measurements
// ==============================================================================
void setMuxChannel(uint8_t channel) {
  digitalWrite(MUX_S0_PIN, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_S1_PIN, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_S2_PIN, (channel & 0x04) ? HIGH : LOW);
  digitalWrite(MUX_S3_PIN, (channel & 0x08) ? HIGH : LOW);
  // Small delay to allow multiplexer signals to settle
  delayMicroseconds(10);
}

float readUltrasonicSensor() {
  // Trigger sensor
  digitalWrite(MUX_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(MUX_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(MUX_TRIG_PIN, LOW);
  
  // Measure pulse duration (timeout after 15ms / ~2.5 meters max range)
  long duration = pulseIn(MUX_ECHO_PIN, HIGH, 15000);
  if (duration == 0) {
    return 3.0f; // Timeout / No echo detected (return out-of-range value)
  }
  
  // Distance in meters = (duration / 2) * speed of sound (343 m/s)
  // (duration / 2) * 0.000343
  float distance = duration * 0.0001715f;
  return distance;
}

// ==============================================================================
// Main Arduino Setup & Loop
// ==============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing ESP32 Proximity (ESP32-A) Node with Dual Multiplexers...");

  // Init Multiplexer Select Pins
  pinMode(MUX_S0_PIN, OUTPUT);
  pinMode(MUX_S1_PIN, OUTPUT);
  pinMode(MUX_S2_PIN, OUTPUT);
  pinMode(MUX_S3_PIN, OUTPUT);
  
  // Init Common Trig & Echo Pins
  pinMode(MUX_TRIG_PIN, OUTPUT);
  digitalWrite(MUX_TRIG_PIN, LOW);
  pinMode(MUX_ECHO_PIN, INPUT);

  // Init CAN interface
  initCAN();
}

void loop() {
  unsigned long now = millis();

  // 1. Send periodic Heartbeat (1Hz)
  if (now - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_time = now;
    
    // Heartbeat payload: [Status (1B)] [Uptime in seconds (4B)]
    uint8_t payload[5];
    payload[0] = 0x00; // Status OK
    uint32_t uptime_sec = now / 1000;
    memcpy(&payload[1], &uptime_sec, 4);
    
    sendCANFrame(0x01, payload, 5);
  }

  // 2. Read sensors sequentially (non-blocking scheduler)
  if (now - last_sensor_trigger_time >= SENSOR_INTERVAL_MS) {
    last_sensor_trigger_time = now;
    
    // Set multiplexers to the current sensor channel
    setMuxChannel(current_sensor_index);
    
    // Read the current sensor through the multiplexer
    float raw_dist = readUltrasonicSensor();
    
    // Apply Low-Pass Filter (Exponential Moving Average)
    if (sensor_distances[current_sensor_index] < 0.0f) {
      sensor_distances[current_sensor_index] = raw_dist; // Initialize on first valid read
    } else {
      sensor_distances[current_sensor_index] = (LPF_ALPHA * raw_dist) + ((1.0f - LPF_ALPHA) * sensor_distances[current_sensor_index]);
    }

    // Send data over CAN: every 2 sensors are grouped into one 8-byte frame (two 32-bit floats)
    // Frame indices map:
    // Sensor 0 & 1 -> cmd_id 0x02
    // Sensor 2 & 3 -> cmd_id 0x03
    // Sensor 4 & 5 -> cmd_id 0x04
    // Sensor 6 & 7 -> cmd_id 0x05
    // Sensor 8 & 9 -> cmd_id 0x06
    if (current_sensor_index % 2 == 1) {
      uint8_t cmd_id = 0x02 + (current_sensor_index / 2);
      float v1 = sensor_distances[current_sensor_index - 1];
      float v2 = sensor_distances[current_sensor_index];
      
      uint8_t payload[8];
      memcpy(&payload[0], &v1, 4);
      memcpy(&payload[4], &v2, 4);
      
      sendCANFrame(cmd_id, payload, 8);
    }
    
    // Increment to next sensor
    current_sensor_index = (current_sensor_index + 1) % NUM_SENSORS;
  }
}
