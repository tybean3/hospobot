#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"

// ==============================================================================
// Hardware Configurations for ESP32 38-Pin DevKit & TJA1051T/3
// ==============================================================================
#define CAN_TX_PIN GPIO_NUM_22      // TWAI/CAN TX Pin
#define CAN_RX_PIN GPIO_NUM_21      // TWAI/CAN RX Pin
#define CAN_STANDBY_PIN GPIO_NUM_4  // TJA1051T/3 S pin (Standby: active HIGH, Normal: LOW)

// I2C pins for INA226 (CAN uses GPIO 21 and 22, so we choose alternative I2C pins)
#define I2C_SDA_PIN 23
#define I2C_SCL_PIN 19

// INA226 I2C Address (default: 0x40)
#define INA226_ADDR 0x40

// CAN Node Configuration
#define NODE_ID 0x1C  // ESP32-C (Core Node)

// Timing configurations
#define HEARTBEAT_INTERVAL_MS 1000
#define TX_INTERVAL_MS 100         // 10Hz update rate

// Actuator Pin Definitions (LED filaments 7-8)
#define NUM_FILAMENTS 2
const uint8_t FILAMENT_PINS[NUM_FILAMENTS] = {12, 13};
const uint8_t LEDC_CHANNELS[NUM_FILAMENTS] = {0, 1};

// Timing states
unsigned long last_tx_time = 0;
unsigned long last_heartbeat_time = 0;

// INA226 configuration values
float shunt_resistor = 0.002f; // 2 milliohms shunt resistor
float current_lsb = 0.001f;    // 1mA LSB

// ==============================================================================
// INA226 Helpers
// ==============================================================================
void writeINA226Register(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

uint16_t readINA226Register(uint8_t reg) {
  Wire.beginTransmission(INA226_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  
  Wire.requestFrom(INA226_ADDR, 2);
  if (Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    return val;
  }
  return 0xFFFF;
}

bool initINA226() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400kHz speed
  
  // Test connection by reading configuration register (default: 0x4127)
  uint16_t config = readINA226Register(0x00);
  if (config == 0xFFFF) {
    Serial.println("INA226 connection failed. Running in simulated power mode.");
    return false;
  }
  
  // Configure INA226: 16 samples average, 1.1ms conversion time for shunt & bus voltage
  writeINA226Register(0x00, 0x4527); 
  
  // Set calibration register: Cal = 0.00512 / (Current_LSB * Shunt)
  // Cal = 0.00512 / (0.001 * 0.002) = 2560 (0x0A00)
  writeINA226Register(0x05, 0x0A00);
  
  Serial.println("INA226 initialized successfully.");
  return true;
}

void getPowerData(float &voltage, float &current) {
  // Check if INA226 is present, otherwise simulate
  uint16_t config = readINA226Register(0x00);
  if (config == 0xFFFF) {
    // Simulated power data
    float elapsed_sec = millis() / 1000.0f;
    voltage = 24.0f + 0.5f * sin(elapsed_sec);
    current = 2.0f + 0.8f * sin(elapsed_sec * 2.0f);
    return;
  }
  
  // Read Bus Voltage register (0x02) - LSB is 1.25mV
  uint16_t raw_voltage = readINA226Register(0x02);
  voltage = raw_voltage * 0.00125f;
  
  // Read Current register (0x04) - LSB is 1mA (0.001A)
  int16_t raw_current = (int16_t)readINA226Register(0x04);
  current = raw_current * 0.001f;
}

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
    Serial.printf("CAN Transmit failed: 0x%X\n", err);
  }
}

// ==============================================================================
// Actuator Controls
// ==============================================================================
void setFilament(uint8_t index, uint8_t mode, uint8_t brightness) {
  if (index >= NUM_FILAMENTS) return;
  
  if (mode == 0) {
    ledcWrite(LEDC_CHANNELS[index], 0);
  } else {
    ledcWrite(LEDC_CHANNELS[index], brightness);
  }
}

void handleIncomingCAN() {
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, 0) == ESP_OK) {
    uint8_t rx_node_id = (rx_msg.identifier >> 5) & 0x3F;
    uint8_t cmd_id = rx_msg.identifier & 0x1F;

    if (rx_node_id == NODE_ID) {
      if (cmd_id == 0x0A) { // LED Filaments 7-8
        if (rx_msg.data_length_code >= 4) {
          Serial.println("Received command for LED Filaments 7-8");
          for (int i = 0; i < 2; i++) {
            uint8_t mode = rx_msg.data[i * 2];
            uint8_t brightness = rx_msg.data[i * 2 + 1];
            setFilament(i, mode, brightness);
          }
          
          // Parrot back filaments 7-8 state on cmd 0x0E to verify function
          sendCANFrame(0x0E, rx_msg.data, 4);
        }
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
  Serial.println("Initializing ESP32 Core Node (ESP32-C)...");

  // PWM setup for LED filaments 7-8
  for (int i = 0; i < NUM_FILAMENTS; i++) {
    ledcSetup(LEDC_CHANNELS[i], 5000, 8); // 5kHz frequency, 8-bit resolution
    ledcAttachPin(FILAMENT_PINS[i], LEDC_CHANNELS[i]);
    ledcWrite(LEDC_CHANNELS[i], 0); // Start off
  }

  // Init INA226 I2C power sensor
  initINA226();

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

  // 3. Send periodic Power Data (10Hz)
  if (now - last_tx_time >= TX_INTERVAL_MS) {
    last_tx_time = now;
    
    float voltage = 0.0f;
    float current = 0.0f;
    getPowerData(voltage, current);
    
    uint8_t payload[8];
    memcpy(&payload[0], &voltage, 4);
    memcpy(&payload[4], &current, 4);
    
    sendCANFrame(0x02, payload, 8);
  }
}
