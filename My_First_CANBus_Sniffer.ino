// requires https://github.com/collin80/esp32_can and https://github.com/collin80/can_common

#include <esp32_can.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <cmath>
#include <stdio.h>
#include <iostream>

// Your gauge MAC address
uint8_t ScreenAddress[] = {0x30, 0xed, 0xa0, 0x21, 0xa6, 0xac}; 

const int send_data_interval = 100;  // 10 times per second

esp_now_peer_info_t peerInfo;

typedef struct struct_sending_data {
  uint8_t flag;
  uint8_t speed_mph;
  uint16_t rpm;
  uint8_t coolant_temp;
  bool indicating_left;
  bool indicating_right;
}struct_sending_data;

struct_sending_data SendingData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // enable for error checking
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send Success" : "Send Failed");
}

void Send_Data(void *parameter) {
  while(true) {
    // no await reply
    esp_now_send(ScreenAddress, (uint8_t *)&SendingData, sizeof(SendingData));
    vTaskDelay(send_data_interval / portTICK_PERIOD_MS);
  }
}

void WiFi_Init(void) {
  // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    
    esp_now_register_send_cb(OnDataSent);

    // Add speedo peer
    peerInfo.channel = 0; // Default channel
    peerInfo.encrypt = false;

    memcpy(peerInfo.peer_addr, ScreenAddress, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // add additional peers here

    Serial.println("ESP-NOW Initialized");
}

void CANBus_Init(void) {
    CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5); // CANBus pins
    CAN0.begin(500000); // 500Kbps
    CAN0.watchFor();
}

uint16_t Process_Endian(uint8_t byte_msb, uint8_t byte_lsb) {
  return (byte_msb << 8) | byte_lsb;
}

bool Process_Bit(uint8_t byte, uint8_t bit_pos) {
  return (byte >> bit_pos) & 0x01;
}

void Process_Coolant_Temp(uint8_t *byte_data) {
    // byte 0
    // mod -40

    int byte_pos = 0;
    int final_temp = byte_data[byte_pos] - 40;

    SendingData.coolant_temp = final_temp;
}

void Process_Speed(uint8_t *byte_data) {
  // in kmph
  // scale 0.01
  // MSB 1
  // LSB 0

  int SPEED_LSB = 0;
  int SPEED_MSB = 1;
  float scale = 0.01;
  float mph_ratio = 0.621;
  float safety_buffer = 1.02;

  float conversion_ratio = scale * safety_buffer;

  uint16_t raw_value = Process_Endian(byte_data[SPEED_MSB], byte_data[SPEED_LSB]);
  float raw_kmph = raw_value * conversion_ratio;
  uint8_t final_mph = std::round(raw_kmph * mph_ratio);

  SendingData.speed_mph = final_mph;
}

void Process_Indicators(uint8_t *byte_data) {
  uint8_t byte_pos = 1;
  uint8_t LEFT_BIT = 2;
  uint8_t RIGHT_BIT = 1;

  uint8_t single_byte = byte_data[byte_pos];

  bool indicator_left_on = Process_Bit(single_byte, LEFT_BIT);
  bool indicator_right_on = Process_Bit(single_byte, RIGHT_BIT);

  SendingData.indicating_left = indicator_left_on;
  SendingData.indicating_right = indicator_right_on;
}

void Process_RPM(uint8_t *byte_data) {
  // scale 3
  // MSB 4
  // LSB 3

  int RPM_LSB = 3;
  int RPM_MSB = 4;
  int scale = 3;

  uint16_t raw_value = Process_Endian(byte_data[RPM_MSB], byte_data[RPM_LSB]);
  uint16_t final_rpm = raw_value * scale;

  SendingData.rpm = final_rpm;
}

void setup() {
    Serial.begin(115200);

    WiFi_Init();
    CANBus_Init();

    SendingData.flag = 0; // define sender ID

    xTaskCreate(Send_Data, "Send_Data", 4096, NULL, 2, NULL);
}


void loop() {
    CAN_FRAME can_message;

    if (CAN0.read(can_message)) {     
      switch (can_message.id) {
        case 0x60D:
          Process_Indicators(can_message.data.byte);
          break;
        case 0x354:
          Process_Speed(can_message.data.byte);
          break;
        case 0x23D:
          Process_RPM(can_message.data.byte);
          break;
        case 0x551:
          Process_Coolant_Temp(can_message.data.byte);
          break;
        default:
          break;
      }
    }
}