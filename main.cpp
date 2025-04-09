#include <Arduino.h>

// Cấu hình chân GPIO
#define RS485_DE_RE_GPIO  4
#define UART_TXD_PIN      17
#define UART_RXD_PIN      18

// Cấu hình giao tiếp
#define BAUDRATE          115200
#define SLAVE_ID          0x01
#define FRAME_TYPE_ENABLE 0x2A
#define HEADER            0xAA
#define TAIL              0xEE
#define STUFFING_BYTE     0xAA

HardwareSerial RS485Serial(1);

// Hàm tính CRC-16 (Modbus)
uint16_t calculateCRC(uint8_t *data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t pos = 0; pos < length; pos++) {
        crc ^= (uint16_t)data[pos];
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void sendServoCommand(bool enable) {
    // Tạo frame data gốc
    uint8_t rawData[7]; // SlaveID(2) + FrameType(1) + DataLen(1) + Data(1) + CRC(2)
    
    // Slave ID (2 byte little-endian)
    rawData[0] = SLAVE_ID & 0xFF;       // Low byte = 0x01
    rawData[1] = (SLAVE_ID >> 8) & 0xFF;// High byte = 0x00
    
    // Frame type
    rawData[2] = FRAME_TYPE_ENABLE;
    
    // Data
    rawData[3] = 0x01; // Data length
    rawData[4] = enable ? 0x01 : 0x00;
    
    // Tính CRC cho 5 byte đầu
    uint16_t crc = calculateCRC(rawData, 5);
    rawData[5] = crc & 0xFF;  // CRC low
    rawData[6] = (crc >> 8) & 0xFF; // CRC high

    // Xử lý byte stuffing
    uint8_t stuffedData[14]; // Tối đa 7*2 = 14 byte
    uint8_t stuffedIndex = 0;
    
    for(uint8_t i=0; i<7; i++){
        stuffedData[stuffedIndex++] = rawData[i];
        if(rawData[i] == HEADER){
            stuffedData[stuffedIndex++] = STUFFING_BYTE;
        }
    }

    // Tạo frame hoàn chỉnh
    uint8_t finalFrame[stuffedIndex + 4];
    finalFrame[0] = HEADER;
    finalFrame[1] = 0xCC;
    memcpy(&finalFrame[2], stuffedData, stuffedIndex);
    finalFrame[stuffedIndex + 2] = HEADER;
    finalFrame[stuffedIndex + 3] = TAIL;

    // Gửi frame
    digitalWrite(RS485_DE_RE_GPIO, HIGH);
    RS485Serial.write(finalFrame, stuffedIndex + 4);
    RS485Serial.flush();
    digitalWrite(RS485_DE_RE_GPIO, LOW);
}

void setup() {
    Serial.begin(115200);
    pinMode(RS485_DE_RE_GPIO, OUTPUT);
    RS485Serial.begin(BAUDRATE, SERIAL_8N1, UART_RXD_PIN, UART_TXD_PIN);

    Serial.println("\nKhởi động hệ thống...");
    
    // Chu trình điều khiển
    Serial.println("[1/2] BẬT servo");
    sendServoCommand(true);
    delay(5000);
    
    Serial.println("[2/2] TẮT servo");
    sendServoCommand(false);
    delay(5000);
    
    Serial.println("HOÀN TẤT CHU TRÌNH");
}

void loop() {}
