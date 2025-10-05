#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <Adafruit_VCNL4200.h>
#include <Adafruit_VoltageSens.h>

// Global structs
Adafruit_VoltageSens volt;
AsyncServer *server = nullptr;

// 🔐 Wi-Fi credentials
const char* ssid = "TMOBILE-6BD8";
const char* password = "eg9rh5np7hk";

// latency related global
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



void print_voltage_reading(sensors_event_t voltage_event){
  Serial.print("Voltage: ");
  Serial.print(voltage_event.voltage);
  Serial.println(" V");
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
  initialize_mpu6050();
  initialize_voltage_sensor();
  calibrate_mpu_6050();

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

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    print_mpu_readings(a, g, temp);

    sensors_event_t voltage_event;
    volt.getEvent(&voltage_event);
    print_voltage_reading(voltage_event);

    Serial.println("");
  }
}