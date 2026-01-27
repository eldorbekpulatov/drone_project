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
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  // bmp.setOutputDataRate(BMP3_ODR_100_HZ); // because in FORCED mode
  Serial.println("Adafruit BMP-388 initialized successfully!");
}

void initialize_icm20948(){
  // Try to initialize!
  if (!icm.begin_I2C(0x69, &Wire1)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) { delay(10); }
  }
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_5_7_HZ);
 
  icm.setAccelRateDivisor(10); // 102.3 Hz
  icm.setGyroRateDivisor(10); // 102.3 Hz
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

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


float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

void calibrate_icm() {
    const int N = 50;
    sensors_event_t a, g, m, t;
    for (int i = 0; i < N; i++) {
        icm.getEvent(&a, &g, &t, &m);
        gyro_bias_x += g.gyro.x;
        gyro_bias_y += g.gyro.y;
        gyro_bias_z += g.gyro.z;
        delay(10);
    }
    gyro_bias_x /= N;
    gyro_bias_y /= N;
    gyro_bias_z /= N;
}


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

  calibrate_icm();

  Serial.println("");
  delay(100); // blocking
}

float prev_roll_rad = 0.0f;
float prev_pitch_rad = 0.0f;

/* Returns integrated roll angle in degreed */
float integrate_roll(sensors_event_t* gyro) {
  float dt = 0.020f;  // 20 ms
  prev_roll_rad += (gyro->gyro.y) * dt;
  return degrees(prev_roll_rad);
}

/* Returns integrated pitch angle in degrees */
float integrate_pitch(sensors_event_t* gyro) {
  float dt = 0.020f;  // 20 ms
  prev_pitch_rad += (gyro->gyro.x) * dt;
  return degrees(prev_pitch_rad);
}

/* Returns trigonometric pitch angle in degrees */
float calculate_pitch(sensors_event_t* accel) {
  float sq = (accel->acceleration.y * accel->acceleration.y) +
             (accel->acceleration.z * accel->acceleration.z);
  float body = -accel->acceleration.x / sqrt(sq);
  return degrees(atan(body));
}

/* Returns trigonometric roll angle in degrees */
float calculate_roll(sensors_event_t* accel) {
  float sq = (accel->acceleration.x * accel->acceleration.x) +
             (accel->acceleration.z * accel->acceleration.z);
  float body = accel->acceleration.y / sqrt(sq);
  return degrees(atan(body));
}

// Non-blocking loop using millis() for timing
unsigned long previousMillis = 0; 
const long interval = 20;
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // sensor reads take about 8 milis 
    sensors_event_t press, alt, temp;
    bmp.getEvent(&temp, &press, &alt);
    sensors_event_t accel, gyro, mag, temp2;
    icm.getEvent(&accel, &gyro, &temp2, &mag);

    gyro.gyro.x -= gyro_bias_x;
    gyro.gyro.y -= gyro_bias_y;
    gyro.gyro.z -= gyro_bias_z;

    Serial.print("Tpitch:");
    Serial.print(calculate_pitch(&accel));
    Serial.print(" Troll:");
    Serial.print(calculate_roll(&accel));
    Serial.print(" Ipitch:");
    Serial.print(integrate_pitch(&gyro));
    Serial.print(" Iroll:");
    Serial.println(integrate_roll(&gyro));
  }
}

// void loop() {
//   // sensor reads take about 8 milis 
//   sensors_event_t press, alt, temp;
//   bmp.getEvent(&temp, &press, &alt);
//   sensors_event_t accel, gyro, mag, temp2;
//   icm.getEvent(&accel, &gyro, &temp2, &mag);

//   print(&accel, &gyro, &mag, &temp2, &press, &alt);
//   delay(500); // blocking
// }