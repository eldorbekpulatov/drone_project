#include <WiFi.h>
#include <AsyncTCP.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ICM20948.h>


// Global structs
AsyncServer *server;
Adafruit_BMP3XX bmp;
Adafruit_ICM20948 icm;

// 🔐 Wi-Fi credentials
const char* ssid = "TMOBILE-6BD8";
const char* password = "eg9rh5np7hk";

unsigned long lastTime = 0;
void initialize_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start Async TCP server
  server = new AsyncServer(80);
  server->onClient([](void *arg, AsyncClient *client) {
    Serial.println("📶 New client connected");

    // 🔧 Disable Nagle's Algorithm (send immediately, no delay)
    client->setNoDelay(true);

    // Optional: print client IP
    Serial.print("Client IP: ");
    Serial.println(client->remoteIP());

    // On data received
    client->onData([](void *arg, AsyncClient *client, void *data, size_t len) {
      if (len >= 4) {
        uint8_t *buffer = (uint8_t *)data;

        int8_t leftX  = (int8_t)buffer[0];
        int8_t leftY  = (int8_t)buffer[1];
        int8_t rightX = (int8_t)buffer[2];
        int8_t rightY = (int8_t)buffer[3];

        Serial.print("L:(");
        Serial.print(leftX);
        Serial.print(",");
        Serial.print(leftY);
        Serial.print(")  R:(");
        Serial.print(rightX);
        Serial.print(",");
        Serial.print(rightY);
        Serial.println(")");
      }
      // print latency
      unsigned long now = millis();
      Serial.print("Latency: Δ = ");
      Serial.print(now - lastTime);
      Serial.println(" ms)");
      lastTime = now;
    }, NULL);

    // On disconnect
    client->onDisconnect([](void *arg, AsyncClient *client) {
      Serial.println("❌ Client disconnected");
    }, NULL);

  }, NULL);

  server->begin();
  Serial.println("🚀 Async TCP server started on port 80");
}

void initialize_bmp388(){
  if (!bmp.begin_I2C(0x77, &Wire1)) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1) { delay(10); }
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  bmp.setPowerMode(BMP3_MODE_NORMAL);
  bmp.applySettings();
  Serial.println("Adafruit BMP-388 initialized successfully!");
}

void initialize_icm20948(){
  // Try to initialize!
  if (!icm.begin_I2C(0x69, &Wire1)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) { delay(10); }
  }
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setAccelRateDivisor(4095);
  icm.setGyroRateDivisor(255);
  icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.println("Adafruit ICM-20948 initialized successfully!");
}

void print(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *mag, 
  sensors_event_t *temp, sensors_event_t *press, sensors_event_t *alt){
  char buf[128];

  // Accel
  snprintf(buf, sizeof(buf),
          "\t\tAccel X: %7.2f  \tY: %7.2f  \tZ: %7.2f m/s^2",
          accel->acceleration.x,
          accel->acceleration.y,
          accel->acceleration.z);
  Serial.println(buf);

  // Gyros
  snprintf(buf, sizeof(buf),
          "\t\tGyros X: %7.2f  \tY: %7.2f  \tZ: %7.2f radians/s",
          gyro->gyro.x,
          gyro->gyro.y,
          gyro->gyro.z);
  Serial.println(buf);

  // Magne
  snprintf(buf, sizeof(buf),
          "\t\tMagne X: %7.2f  \tY: %7.2f  \tZ: %7.2f uT",
          mag->magnetic.x,
          mag->magnetic.y,
          mag->magnetic.z);
  Serial.println(buf);

  // Temp / Press / Alt
  snprintf(buf, sizeof(buf),
          "\t\tTemp: %6.2f *C\tPress: %6.2f hPa\tAlt: %8.2f m",
          temp->temperature,
          press->pressure / 100.0F,
          alt->altitude);
  Serial.println(buf);

  Serial.println();
}


// volatile int counter = 0;
// IRAM_ATTR void inc_counter() { counter += 1; }
// attachInterrupt(digitalPinToInterrupt(16), inc_counter, CHANGE); // FG

// static assignment of pins to 0/1
// pinMode(16, OUTPUT); // DIR    
// digitalWrite(6, HIGH); 

// analog write can output PWM signal
// pinMode(0, OUTPUT); // PWM
// analogWrite(0, 127); 

void setup(void) {
  Serial.begin(115200);
  while (!Serial){ delay(10); }
  Serial.println("Serial Initialized!"); 

  initialize_wifi();

  // On Qt Py ESP32-S3, STEMMA QT = SDA1=41, SCL1=40
  // Ref: https://learn.adafruit.com/adafruit-qt-py-esp32-s3
  Wire1.begin(41, 40);
  initialize_bmp388();
  initialize_icm20948();

  Serial.println("");
  delay(100); // blocking
}

// // Non-blocking loop using millis() for timing
// unsigned long previousMillis = 0; 
// const long interval = 500;
// void loop() {
//   unsigned long currentMillis = millis();
//   if (currentMillis - previousMillis >= interval) {
//     previousMillis = currentMillis;
//     // ... do something every 500ms
//   }
// }

void loop() {
  sensors_event_t accel, gyro, mag, temp, press, alt, temp2;
  bmp.getEvent(&temp, &press, &alt); // temperature, pressure, altitude
  icm.getEvent(&accel, &gyro, &temp2, &mag); // accel, gyro, mag, temp2
  print(&accel, &gyro, &mag, &temp, &press, &alt);
  delay(500); // blocking
}