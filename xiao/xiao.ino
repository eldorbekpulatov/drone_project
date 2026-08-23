#include <Arduino.h>


// Core 2 - blinker task
#ifndef LED_BUILTIN
#define LED_BUILTIN 21
#endif
void blinkTask(void *parameter) {
  pinMode(LED_BUILTIN, OUTPUT);

  while (true) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


// Give me access to UART1 and call it MotorSerial
HardwareSerial MotorSerial(1);

#define RX_PIN 44   // D7 (XIAO ESP32-S3)
#define TX_PIN 43   // D6

uint32_t bauds[] = {
  9600,
  19200,
  38400,
  57600,
  74880,
  115200
};
int baudIndex = 0;


void startBaud(uint32_t baud) {
  MotorSerial.end();
  delay(50);

  MotorSerial.begin(baud, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.printf("\n--- Testing baud: %lu ---\n", baud);
}


void setup() {
  Serial.begin(115200);
  
  xTaskCreatePinnedToCore(
    blinkTask,
    "BlinkTask",
    2048,
    nullptr,
    1,
    nullptr,
    0 // core 0
  );

  Serial.println("Starting baud sweep...");
}

void loop() {
  static uint32_t lastSwitch = 0;
  static bool gotData = false;

  // start current baud
  if (lastSwitch == 0) {
    startBaud(bauds[baudIndex]);
    lastSwitch = millis();
  }

  // read UART
  while (MotorSerial.available()) {
    uint8_t b = MotorSerial.read();
    Serial.printf("RX@%lu: 0x%02X\n", bauds[baudIndex], b);
  }


  // switch baud every 2 seconds
  if (millis() - lastSwitch > 2000) {
    baudIndex = (baudIndex + 1) % (sizeof(bauds) / sizeof(bauds[0]));
    startBaud(bauds[baudIndex]);
    lastSwitch = millis();
  }

  delay(10);
}

