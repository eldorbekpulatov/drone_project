#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <Adafruit_VoltageSens.h>
#include <Adafruit_VCNL4200.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ICM20948.h>

// Global structs
AsyncServer *server = nullptr;
Adafruit_VoltageSens volt;
Adafruit_VCNL4200 vcnl;
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

void initialize_voltage_sensor(){
  // Example calibration data for a voltage divider with R1=4.7k and R2=330
  // This maps the ADC reading (0-1023) to the actual voltage (0-1V)
  static const float x_known[] = {0.0f, 0.211f, 0.710f}; // Voltage divider values
  static const float y_known[] = {0.0f, 3.26f, 11.0f};   // Actual voltages
  static const float offsets[] = {0.0f, -0.44f, -0.60f}; // Calibration offsets

  // Initialize the voltage sensor on ADC pin A0 with a max voltage of 1V
  volt = Adafruit_VoltageSens(A0, 1.0, x_known, y_known, offsets, sizeof(x_known)/sizeof(x_known[0]));
  
  Serial.println("Voltage Sensor initialized!");
}

void initialize_vcnl4200(){
  if (!vcnl.begin()) {
    Serial.println("Could not find a valid VCNL4200 sensor, check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("VCNL4200 found!");

  vcnl.setALSshutdown(false);
  vcnl.setALSIntegrationTime(VCNL4200_ALS_IT_100MS);
  vcnl.setALSPersistence(VCNL4200_ALS_PERS_2);
  vcnl.setProxShutdown(false);
  vcnl.setProxHD(false);
  vcnl.setProxLEDCurrent(VCNL4200_LED_I_200MA);
  vcnl.setProxIntegrationTime(VCNL4200_PS_IT_8T);
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
}

void initialize_icm20948(){
  // Try to initialize!
  if (!icm.begin_I2C()) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) { delay(10); }
  }
  Serial.println("ICM20948 Found!");

  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }

  Serial.println("Adafruit ICM-20948 initialized successfully!");
}

void print_voltage_reading(sensors_event_t voltage_event){
  Serial.print("Voltage: ");
  Serial.print(voltage_event.voltage);
  Serial.println(" V");
}

void print_vcnl_readings(uint16_t proxData, uint16_t alsData, uint16_t whiteData){
  Serial.print("Prox Data: ");
  Serial.print(proxData);
  Serial.print(", ALS Data: ");
  Serial.print(alsData);
  Serial.print(", White Data: ");
  Serial.println(whiteData);
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
  initialize_voltage_sensor(); 
  initialize_vcnl4200();
  initialize_bmp388();
  initialize_icm20948();

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
    
    sensors_event_t voltage_event;
    volt.getEvent(&voltage_event);
    print_voltage_reading(voltage_event);

    uint16_t proxData = vcnl.readProxData();
    uint16_t alsData = vcnl.readALSdata();
    uint16_t whiteData = vcnl.readWhiteData();
    print_vcnl_readings(proxData, alsData, whiteData);

    bmp.performReading();
    print_bmp388_readings();

    sensors_event_t accel, gyro, mag, temp;
    icm.getEvent(&accel, &gyro, &temp, &mag);
    print_icm_readings(accel, gyro, temp, mag);

    Serial.println("");
  }
}