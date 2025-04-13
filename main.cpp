#include <Arduino.h>
#include "Communication_Interface.h"  // Bao gồm thư viện giao tiếp với Driver

// --- Cấu hình hằng số ---
#define ON_SERVO 1
#define RETURN_OK 0
#define MAX_RETRIES 3

#define ID_1 1
#define ID_2 2

// --- Danh sách ID Servo ---
const int servoIDs[] = {ID_1, ID_2}; 
const int numServos = sizeof(servoIDs) / sizeof(servoIDs[0]);

// --- Chân GPIO cho RS485 ---
#define RS485_DE_RE_GPIO  4   // DE & RE nối chung 1 chân
#define UART_TXD_PIN      17
#define UART_RXD_PIN      18

// --- Khởi tạo UART RS485 ---
HardwareSerial RS485Serial(1); // Dùng Serial1 (UART1)

// --- Hàm bật chế độ gửi ---
void RS485_WriteEnable() {
  digitalWrite(RS485_DE_RE_GPIO, HIGH); // DE = HIGH: cho phép gửi
  delayMicroseconds(100); // Đợi ổn định
}

// --- Hàm bật chế độ nhận ---
void RS485_ReadEnable() {
  delayMicroseconds(100); // Đợi tín hiệu gửi ổn định xong
  digitalWrite(RS485_DE_RE_GPIO, LOW);  // DE = LOW: cho phép nhận
}

// --- Gửi gói tin qua RS485 ---
void RS485_Send(const uint8_t *data, size_t len) {
  RS485_WriteEnable();
  RS485Serial.write(data, len);
  RS485Serial.flush();       // Đợi gửi hết
  RS485_ReadEnable();        // Trở về chế độ nhận
}

// --- Hàm bật tất cả servo ---
void ENABLE_ALL_SERVO() {
  for (int i = 0; i < numServos; i++) {
    int retryCount = 0;
    delay(50);

    while (SERVO_ServoEnable(servoIDs[i], ON_SERVO) != RETURN_OK) {
      Serial.printf("ENABLE SERVO ID %d - JOINT %d ENABLE AGAIN\n", servoIDs[i], i + 1);
      delay(200);

      if (++retryCount >= MAX_RETRIES) {
        Serial.printf("ENABLE SERVO ID %d FAILED!\n", servoIDs[i]);
        break;
      }
    }
  }
}

void setup() {
  // --- Serial monitor ---
  Serial.begin(115200);
  while (!Serial);

  // --- RS485 GPIO setup ---
  pinMode(RS485_DE_RE_GPIO, OUTPUT);
  digitalWrite(RS485_DE_RE_GPIO, LOW); // Bắt đầu ở chế độ nhận

  // --- UART1 (RS485) setup ---
  RS485Serial.begin(115200, SERIAL_8N1, UART_RXD_PIN, UART_TXD_PIN);

  Serial.println("Starting RS485 Servo Enable Sequence...");
  ENABLE_ALL_SERVO();
}

void loop() {
  // Có thể nhận phản hồi hoặc gửi thêm lệnh nếu cần
}
