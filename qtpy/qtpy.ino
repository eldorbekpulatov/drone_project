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

#define SEALEVELPRESSURE_HPA (1013.25)
void initialize_bmp388(){
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1) { delay(10); }
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  Serial.println("Adafruit BMP-388 initialized successfully!");
}


void initialize_icm20948(){
  // Try to initialize!
  if (!icm.begin_I2C()) {
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

  

void print_bmp388_readings(){
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure() / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
}

void print_icm_readings(sensors_event_t accel, sensors_event_t gyro, sensors_event_t temp, sensors_event_t mag){
  Serial.print("\t\tTemperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("\t\tMag X: ");
  Serial.print(mag.magnetic.x);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z);
  Serial.println(" uT");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();
}


// volatile int counter = 0;
// IRAM_ATTR void inc_counter() {
//     counter += 1;
// }
// attachInterrupt(digitalPinToInterrupt(16), inc_counter, CHANGE); // FG

// pinMode(16, OUTPUT); // DIR    
// digitalWrite(6, HIGH); 

// pinMode(0, OUTPUT); // PWM
// analogWrite(0, 127); 

void setup(void) {
  Serial.begin(115200);
  while (!Serial){ delay(10); }
  Serial.println("Serial Initialized!"); 

  // Try to initialize!
  initialize_wifi();
  // initialize_icm20948();
  // initialize_bmp388();

  Serial.println("");
  delay(100); // blocking
}

// use millis() so that loop itself becomes non-blocking
unsigned long previousMillis = 0; 
const long interval = 250; 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // bmp.performReading();
    // print_bmp388_readings();

    // sensors_event_t accel, gyro, mag, temp;
    // icm.getEvent(&accel, &gyro, &temp, &mag);
    // print_icm_readings(accel, gyro, temp, mag);

    Serial.println("");
  }
}
