#include <Arduino.h>

#define RS485_DE_RE_GPIO  4     // Chân GPIO điều khiển chế độ truyền/nhận của RS485
#define UART_TXD_PIN      17    // Chân TX của ESP32 (gửi dữ liệu)
#define UART_RXD_PIN      18    // Chân RX của ESP32 (nhận dữ liệu)

void setup() {
  Serial.begin(115200);       // Khởi tạo Serial Monitor với baudrate 115200
  Serial2.begin(115200, SERIAL_8N1, UART_RXD_PIN, UART_TXD_PIN);  // Khởi tạo Serial2 (RS485)

  pinMode(RS485_DE_RE_GPIO, OUTPUT);  // Cấu hình GPIO để điều khiển chế độ truyền/nhận của RS485

  // Chuỗi dữ liệu cần gửi dưới dạng hex
  uint8_t dataToSend[] = {0xAA, 0xCC, 0x01, 0x2A, 0x01, 0xA0, 0x01, 0xAA, 0xEE};
  
  Serial.println("Đang gửi chuỗi hex qua RS485...");

  // Chuyển sang chế độ truyền (RS485)
  digitalWrite(RS485_DE_RE_GPIO, HIGH);
  delay(10);  // Chờ ổn định

  // Gửi dữ liệu qua Serial2 (RS485)
  Serial2.write(dataToSend, sizeof(dataToSend));  // Gửi dữ liệu qua RS485

  Serial.println("Đã gửi xong!");

  delay(10);  // Đợi dữ liệu ra hết
  digitalWrite(RS485_DE_RE_GPIO, LOW);  // Chuyển lại chế độ nhận (RS485)
}

void loop() {
  // Không cần làm gì trong loop
}
