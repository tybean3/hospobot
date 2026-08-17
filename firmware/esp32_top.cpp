#include <Arduino.h>
#include "driver/twai.h"

// ==============================================================================
// Hardware Configurations for ESP32 38-Pin DevKit & TJA1051T/3
// ==============================================================================
#define CAN_TX_PIN GPIO_NUM_22      // TWAI/CAN TX Pin
#define CAN_RX_PIN GPIO_NUM_21      // TWAI/CAN RX Pin
#define CAN_STANDBY_PIN GPIO_NUM_4  // TJA1051T/3 S pin (Standby: active HIGH, Normal: LOW)

// CAN Node Configuration
#define NODE_ID 0x1B  // ESP32-B (Top Node)

// Timing configurations
#define HEARTBEAT_INTERVAL_MS 1000
#define TX_INTERVAL_MS 100         // 10Hz update rate

// Sensor Pin Definitions
#define BEAM_BREAK_PIN 34           // Input for Beam Break sensor
#define DRAW_LOCK_PIN 33            // Output for solenoid/motor drawer lock

// Actuator Pin Definitions (LED filaments 1-6)
#define NUM_FILAMENTS 6
const uint8_t FILAMENT_PINS[NUM_FILAMENTS] = {12, 13, 14, 25, 26, 27};
const uint8_t LEDC_CHANNELS[NUM_FILAMENTS] = {0, 1, 2, 3, 4, 5};

// Simulated RFID State
uint32_t current_rfid_uid = 0x00000000;
unsigned long last_rfid_seen_time = 0;

// Timing states
unsigned long last_tx_time = 0;
unsigned long last_heartbeat_time = 0;

// ==============================================================================
// CAN / TWAI Helpers
// ==============================================================================
void initCAN() {
  pinMode(CAN_STANDBY_PIN, OUTPUT);
  digitalWrite(CAN_STANDBY_PIN, LOW); // Pull LOW to enable normal mode on TJA1051T/3

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500kbps CAN Bus Bitrate
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI Driver installed successfully");
  } else {
    Serial.println("Failed to install TWAI Driver");
    return;
  }

  if (twai_start() == ESP_OK) {
    Serial.println("TWAI Driver started successfully");
  } else {
    Serial.println("Failed to start TWAI Driver");
  }
}

void sendCANFrame(uint8_t cmd_id, const uint8_t *data, uint8_t dlc) {
  twai_message_t tx_msg;
  tx_msg.identifier = (NODE_ID << 5) | (cmd_id & 0x1F); // 11-bit standard ID
  tx_msg.extd = 0;
  tx_msg.rtr = 0;
  tx_msg.data_length_code = dlc;
  
  if (data != nullptr && dlc > 0) {
    memcpy(tx_msg.data, data, dlc);
  }
  
  esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
  if (err != ESP_OK) {
    Serial.printf("CAN Transmit failed on cmd 0x%X: 0x%X\n", cmd_id, err);
  }
}

// ==============================================================================
// Actuator Controls
// ==============================================================================
void setFilament(uint8_t index, uint8_t mode, uint8_t brightness) {
  if (index >= NUM_FILAMENTS) return;
  
  // mode = 0: Off, 1: Steady, 2: Pulse/Blink
  if (mode == 0) {
    ledcWrite(LEDC_CHANNELS[index], 0);
  } else {
    // Mode is steady or active, write brightness (mapped 0-255)
    ledcWrite(LEDC_CHANNELS[index], brightness);
  }
}

void handleIncomingCAN() {
  twai_message_t rx_msg;
  // Non-blocking check for incoming frames
  if (twai_receive(&rx_msg, 0) == ESP_OK) {
    uint8_t rx_node_id = (rx_msg.identifier >> 5) & 0x3F;
    uint8_t cmd_id = rx_msg.identifier & 0x1F;

    // Check if target is ESP32-B (this node)
    if (rx_node_id == NODE_ID) {
      switch (cmd_id) {
        case 0x0A: // LED Filaments 1-4
          if (rx_msg.data_length_code >= 8) {
            Serial.println("Received command for LED Filaments 1-4");
            for (int i = 0; i < 4; i++) {
              uint8_t mode = rx_msg.data[i * 2];
              uint8_t brightness = rx_msg.data[i * 2 + 1];
              setFilament(i, mode, brightness);
            }
            
            // Parrot back filaments 1-4 state on cmd 0x0E to verify function
            sendCANFrame(0x0E, rx_msg.data, 8);
          }
          break;

        case 0x0B: // LED Filaments 5-6
          if (rx_msg.data_length_code >= 4) {
            Serial.println("Received command for LED Filaments 5-6");
            for (int i = 0; i < 2; i++) {
              uint8_t mode = rx_msg.data[i * 2];
              uint8_t brightness = rx_msg.data[i * 2 + 1];
              setFilament(4 + i, mode, brightness);
            }
            
            // Parrot back filaments 5-6 state on cmd 0x0F to verify function
            sendCANFrame(0x0F, rx_msg.data, 4);
          }
          break;

        case 0x0C: // LED indicators
          if (rx_msg.data_length_code >= 2) {
            uint16_t mask = (rx_msg.data[1] << 8) | rx_msg.data[0];
            Serial.printf("Received LED Indicator bitmask: 0x%04X\n", mask);
            // Apply to hardware indicators (GPIOs, NeoPixels, etc.)
          }
          break;

        case 0x0D: // Draw Lock
          if (rx_msg.data_length_code >= 1) {
            bool lock = rx_msg.data[0] > 0;
            Serial.printf("Received Draw Lock command: %s\n", lock ? "LOCKED" : "UNLOCKED");
            digitalWrite(DRAW_LOCK_PIN, lock ? HIGH : LOW);
          }
          break;

        default:
          break;
      }
    }
  }
}

// ==============================================================================
// Main Arduino Setup & Loop
// ==============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing ESP32 Top Node (ESP32-B)...");

  // Actuator pin setups
  pinMode(BEAM_BREAK_PIN, INPUT);
  pinMode(DRAW_LOCK_PIN, OUTPUT);
  digitalWrite(DRAW_LOCK_PIN, LOW); // Default unlocked

  // PWM configurations for filaments
  for (int i = 0; i < NUM_FILAMENTS; i++) {
    ledcSetup(LEDC_CHANNELS[i], 5000, 8); // 5kHz frequency, 8-bit resolution
    ledcAttachPin(FILAMENT_PINS[i], LEDC_CHANNELS[i]);
    ledcWrite(LEDC_CHANNELS[i], 0); // Start off
  }

  // CAN Setup
  initCAN();
}

void loop() {
  unsigned long now = millis();

  // 1. Process incoming CAN messages
  handleIncomingCAN();

  // 2. Send Heartbeat (1Hz)
  if (now - last_heartbeat_time >= HEARTBEAT_INTERVAL_MS) {
    last_heartbeat_time = now;
    
    uint8_t payload[5];
    payload[0] = 0x00; // Status OK
    uint32_t uptime_sec = now / 1000;
    memcpy(&payload[1], &uptime_sec, 4);
    
    sendCANFrame(0x01, payload, 5);
  }

  // 3. Send periodic Telemetry (10Hz)
  if (now - last_tx_time >= TX_INTERVAL_MS) {
    last_tx_time = now;
    
    // --- Send Beam break & RFID Tag ---
    uint8_t beam_state = digitalRead(BEAM_BREAK_PIN) == HIGH ? 1 : 0;
    
    // Simulating RFID scan for test (e.g. tag 0xA4F28D10 scanned periodically)
    if (now % 10000 < 100) {
      current_rfid_uid = 0xA4F28D10;
      last_rfid_seen_time = now;
    } else if (now - last_rfid_seen_time > 2000) {
      current_rfid_uid = 0x00000000; // Reset after 2 seconds
    }

    uint8_t payload[5];
    payload[0] = beam_state;
    memcpy(&payload[1], &current_rfid_uid, 4);
    sendCANFrame(0x02, payload, 5);

    // --- Send Proximity Test (oscillating float value) ---
    float elapsed_sec = now / 1000.0f;
    float test_dist = 1.325f + 1.175f * sin(0.5f * elapsed_sec); // Oscillate between 0.15m and 2.5m
    
    uint8_t prox_payload[4];
    memcpy(prox_payload, &test_dist, 4);
    sendCANFrame(0x03, prox_payload, 4);
  }
}
