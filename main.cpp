#include <Arduino.h>
#include "Communication_Setting_ESP32.h"
#include "Communication_Interface.h"

//	DEFINE STATE
#define ON_SERVO 1
#define OFF_SERVO 0
#define RETURN_OK 0
#define CW 1
#define CCW 0
#define ACCELERATION_TIME_PARAMETER 3
#define DECELERATION_TIME_PARAMETER 4
#define DELAY_TICK_ENABLE_SERVO 100
#define DELAY_TICK_DISABLE_SERVO 100

//	DEFINE SERVO ID
#define ID_1 1
// #define ID_2 2

// BUZZER
#define BUZZER 9

// Phiên bản KHÔNG CHẶN chương trình (non-blocking)
void ENABLE_BUZZER(unsigned int time)
{
    static unsigned long buzzerStartTime = 0;
    static bool buzzerState = false;

    if (!buzzerState) // Nếu buzzer chưa bật
    {
        digitalWrite(BUZZER, HIGH);
        buzzerStartTime = millis();
        buzzerState = true;
    }

    if (buzzerState && millis() - buzzerStartTime >= time) // Nếu đã đủ thời gian
    {
        digitalWrite(BUZZER, LOW);
        buzzerState = false;
    }
}

// Hàm điều khiển bật/tắt servo
void ENABLE_ALL_SERVO()
{
    const int servoIDs[] = {ID_1};
    const int numServos = sizeof(servoIDs) / sizeof(servoIDs[0]);

    for (int i = 0; i < numServos; i++)
    {
        int retryCount = 0;
        ENABLE_BUZZER(DELAY_TICK_ENABLE_SERVO);
        delay(50);

        while (SERVO_ServoEnable(servoIDs[i], ON_SERVO) != RETURN_OK)
        {
            Serial.printf("ENABLE SERVO ID %d - JOINT %d ENABLE AGAIN\n", servoIDs[i], i + 1);
            delay(200);
            if (++retryCount >= 10)
            {
                Serial.println("ENABLE SERVO ID " + String(servoIDs[i]) + " FAILED!");
                break;
            }
        }
    }
}

void DISABLE_ALL_SERVO()
{
    const int servoIDs[] = {ID_1};
    const int numServos = sizeof(servoIDs) / sizeof(servoIDs[0]);

    for (int i = 0; i < numServos; i++)
    {
        int retryCount = 0;
        ENABLE_BUZZER(DELAY_TICK_DISABLE_SERVO);
        delay(50);

        while (SERVO_ServoEnable(servoIDs[i], OFF_SERVO) != RETURN_OK)
        {
            Serial.printf("DISABLE SERVO ID %d - JOINT %d DISABLE AGAIN\n", servoIDs[i], 6 - i);
            delay(200);
            if (++retryCount >= 10)
            {
                Serial.println("DISABLE SERVO ID " + String(servoIDs[i]) + " FAILED!");
                break;
            }
        }
    }
}

void setup() {
  Serial.begin(115200);       // Khởi tạo Serial Monitor với baudrate 115200
  RS485_SETUP();
  pinMode(BUZZER, OUTPUT);

  Serial.println("Đang khởi tạo hệ thống bật servo qua RS485...");
  ENABLE_ALL_SERVO();

  delay(5000);
  DISABLE_ALL_SERVO();

}

void loop() {
  // Không làm gì trong loop vì chỉ cần thực hiện một lần trong setup
}
