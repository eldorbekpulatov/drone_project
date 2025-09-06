#include <Adafruit_MPU6050.h>
#include <Adafruit_VCNL4200.h>
#include <Adafruit_VoltageSens.h>

// Global structs
Adafruit_VCNL4200 vcnl;
Adafruit_MPU6050 mpu;
Adafruit_VoltageSens volt;


void initialize_mpu6050(){
  if (!mpu.begin()) {
    Serial.println("Could not find a valid  MPU6050 sensor, check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
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

void initialize_voltage_sensor(){
  // Example calibration data for a voltage divider with R1=4.7k and R2=330
  // This maps the ADC reading (0-1023) to the actual voltage (0-1V)
  static const float x_known[] = {0.0f, 0.210f, 0.710f}; // Voltage divider values
  static const float y_known[] = {0.0f, 3.266f, 11.0f};  // Actual voltages
  static const float offsets[] = {0.0f, -0.50f, -0.60f}; // Calibration offsets
  
  // Initialize the voltage sensor on ADC pin A0 with a max voltage of 1V
  volt = Adafruit_VoltageSens(A0, 1.0, x_known, y_known, offsets, sizeof(x_known)/sizeof(x_known[0]));
  
  Serial.println("Voltage Sensor initialized!");
}


void print_mpu_readings(sensors_event_t a, sensors_event_t g, sensors_event_t temp){
  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");
}


void print_vcnl_readings( uint16_t proxData, uint16_t alsData, uint16_t whiteData){
  Serial.print("Prox Data: ");
  Serial.print(proxData);
  Serial.print(", ALS Data: ");
  Serial.print(alsData);
  Serial.print(", White Data: ");
  Serial.println(whiteData);
}

void print_voltage_reading(sensors_event_t voltage_event){
  /* Print out the values */
  Serial.print("Voltage: ");
  Serial.print(voltage_event.voltage);
  Serial.println(" V");
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial){
    delay(10); // will pause until serial console opens
  }
  Serial.println("Serial Initialized!");

  // Try to initialize!
  initialize_mpu6050();
  initialize_vcnl4200();
  initialize_voltage_sensor();

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  print_mpu_readings(a, g, temp);

  // Read the proximity sensor data
  uint16_t proxData = vcnl.readProxData();
  // Read the ambient light sensor (ALS) data
  uint16_t alsData = vcnl.readALSdata();
  // Read the raw white sensor data
  uint16_t whiteData = vcnl.readWhiteData();
  print_vcnl_readings(proxData, alsData, whiteData);

  // Read the voltage sensor data
  sensors_event_t voltage_event;
  volt.getEvent(&voltage_event);
  print_voltage_reading(voltage_event);

  Serial.println("");
  delay(500);
}