#include <Adafruit_MPU6050.h>
#include <Adafruit_VCNL4200.h>
#include <Adafruit_VoltageSens.h>

// Global structs
Adafruit_VoltageSens volt;

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


void setup(void) {
  Serial.begin(115200);
  while (!Serial){
    delay(10); // will pause until serial console opens
  }
  Serial.println("Serial Initialized!");

  // Try to initialize!
  initialize_voltage_sensor();

  Serial.println("");
  delay(100);
}

void loop() {
  // Read the voltage sensor data
  sensors_event_t voltage_event;
  volt.getEvent(&voltage_event);

  /* Print out the values */
  Serial.print("Extrapolated Voltage: ");
  Serial.print(voltage_event.voltage);
  Serial.println(" V");

  int raw = analogRead(A0); // ADC reading (0-1023 for 10-bit 1V max)
  float vdiv = (raw / 1023.0f) * 1.0f; // 1.0V max for voltage divider
  float actualVoltage = vdiv * (4.68 + 0.326) / 0.326; // R1=4.7k, R2=330
  Serial.print("Voltage Divider approx: ");
  Serial.print(actualVoltage);
  Serial.println(" V");

  Serial.println("");
  delay(500);
}