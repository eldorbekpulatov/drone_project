#include <Adafruit_MPU6050.h>
#include <Adafruit_VCNL4200.h>
#include <Adafruit_VoltageSens.h>

// Global structs
Adafruit_VCNL4200 vcnl;
Adafruit_MPU6050 mpu;
Adafruit_VoltageSens volt;

// https://adafruit.github.io/Adafruit_MPU6050/html/class_adafruit___m_p_u6050.html
void initialize_mpu6050(){
  if (!mpu.begin()) {
    Serial.println("Could not find a valid  MPU6050 sensor, check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found!");
  
  /*
    ---------------------------------------------------------------
    MPU-6050 Clock Source Selection
    ---------------------------------------------------------------
    The MPU-6050's internal clock source selection affects both its
    output stability and the performance of low-power cycle mode.
    Choosing a stable clock source is critical for precise timing.

    Clock source is selected in the PWR_MGMT_1 register (0x6B).

    | Clock Source (CLKSEL)         | Value | Behavior/Notes                                        | Impact on Output Rate                                           |
    |-------------------------------|-------|-------------------------------------------------------|-----------------------------------------------------------------|
    | Internal 8 MHz RC oscillator  | 0     | Default after power-up. Less stable, temp sensitive.  | Adequate for basic/low-power modes; may introduce jitter/drift. |
    | PLL with X Gyro reference     | 1     | Uses PLL synced to X gyro.                            | Recommended for best accuracy/stability.                        |
    | PLL with Y Gyro reference     | 2     | Uses PLL synced to Y gyro.                            | High stability and accuracy.                                    |
    | PLL with Z Gyro reference     | 3     | Uses PLL synced to Z gyro.                            | High stability and accuracy.                                    |
    | External 32.768kHz/19.2MHz    | 4-6   | Uses external clock (requires hardware).              | Most accurate, but needs external clock.                        |

    - For most applications, use a PLL gyro reference (1-3) for best timing.
    - Use internal oscillator (0) for lowest power, but expect more drift.
    - External clock (4-6) only if you have the hardware.

    Reference: See Register 107 (PWR_MGMT_1) in the MPU6050 datasheet.
  */
  mpu.setClock(MPU6050_INTR_8MHz);

  /*
    ---------------------------------------------------------------
    Sample Rate Configuration for MPU6050
    ---------------------------------------------------------------
    Sample Rate Configuration:
      - SMPLRT_DIV sets how often the sensor outputs data:
          Sample Rate = Base Rate / (1 + SMPLRT_DIV)
          Base Rate: 1kHz (DLPF enabled), 8kHz (DLPF disabled)
          Example: SMPLRT_DIV=9 → 100Hz output

    - Gyroscope Output Rate:
        * 8 kHz  --> When DLPF is disabled (DLPF_CFG = 0 or 7)
        * 1 kHz  --> When DLPF is enabled  (DLPF_CFG = 1~6)

    - Accelerometer Output Rate: Always 1 kHz

    - Example:
        If SMPLRT_DIV = 255 and DLPF is enabled:
            Sample Rate = 1 kHz / (1 + 255) ≈ 3.9 Hz

    - Reference: See Register 26 in the MPU6050 datasheet for details.
    ---------------------------------------------------------------
  */
  mpu.setSampleRateDivisor(9); // 1KHz / (9+1) = 100Hz

  /*
    Cycle Mode (Low Power):
      - Sensor sleeps between measurements, wakes up at set intervals
      - In cycle mode, SMPLRT_DIV is ignored; use setCycleRate() instead
      - Use cycle mode for low-power, infrequent measurements

    Note:
      - Use either normal mode (SMPLRT_DIV) or cycle mode (setCycleRate), not both.
  */
  mpu.enableCycle(false);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);



  /*
    Choosing the right high-pass filter (HPF) setting for the MPU-6050 depends on your
    motion-detection needs. The HPF is used for motion interrupt logic to filter out
    low-frequency effects (like gravity) and allow faster, transient accelerations
    (such as taps, shakes, or quick movements) to trigger interrupts.

    Guidelines for HPF cutoff frequency selection:

    | Application                        | ACCEL_HPF Value | Cutoff Frequency | Notes                                                                  |
    |------------------------------------|-----------------|------------------|------------------------------------------------------------------------|
    | Simple motion detection            | 0x01 (1)        | 5 Hz             | Ignores slow orientation changes (like tilting), reacts to sharp moves |
    | Detecting freefall                 | 0x04 (4)        | 0.63 Hz          | Good for steady, low-g events; set threshold low for best results      |
    | Sustained motion, ignore vibration | 0x02 (2)        | 2.5 Hz           | Sensitive to slower motion, filters out gravity and vibration          |
    | Zero-motion detection              | 0x04 (4)        | 0.63 Hz          | Most sensitive; filters subtle, slow motion for reliable detection     |

    - Higher cutoff (e.g., 5 Hz): Ignores slow changes, good for quick motion detection.
    - Lower cutoff (e.g., 0.63 Hz): Sensitive to slow or subtle motion, good for freefall or zero-motion detection.
  */
  mpu.setAccelerometerStandby(false, false, false); // enable x,y,z
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_2_5_HZ);

  /*
    Choosing the right Digital Low Pass Filter (DLPF) setting for the MPU-6050 depends on your application's needs.
    The DLPF helps filter out high-frequency noise from the sensor data.

    Guidelines for DLPF cutoff frequency selection:

    | Application                        | DLPF_CFG Value | Cutoff Frequency | Notes                                                                 |
    |------------------------------------|----------------|------------------|-----------------------------------------------------------------------|
    | Fast response, low latency         | 1              | 188 Hz           | Minimal filtering, more noise, good for fast control (e.g., drones)   |
    | Balanced filtering                 | 3 or 4         | 42 Hz / 20 Hz    | Good compromise for most uses                                         |
    | Stable, low-noise readings         | 6              | 5 Hz             | Heavy filtering, higher latency, good for slow robots/orientation     |

    - Lower DLPF_CFG (higher cutoff): Less filtering, faster response, more noise.
    - Higher DLPF_CFG (lower cutoff): More filtering, slower response, smoother data.
  */
  mpu.setGyroStandby(false, false, false); // enable x,y,z
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  /*
    Motion Detection Configuration:
      - Motion detection threshold (in LSB): setMotionDetectionThreshold()
          * 1 LSB = 2 mg (0.002g)
          * Example: threshold=2 → 4 mg
      - Motion detection duration (in ms): setMotionDetectionDuration()
          * Duration = duration LSB (1 kHz rate, LSB=1ms)
          * Example: duration=5 → 5 ms
      - Motion interrupt: setMotionInterrupt(true) to enable

    Note:
      - Motion detection uses the accelerometer data.
      - Choose threshold/duration based on expected motion profile.
  */
  mpu.setMotionInterrupt(false); // enable motion interrupt
  mpu.setMotionDetectionThreshold(2); // 2 LSB = 0.06g
  mpu.setMotionDetectionDuration(5);  // 5 LSB = 5 ms 

  // Other settings
  mpu.setTemperatureStandby(true); // enable temp sensor
  mpu.enableSleep(false);
  mpu.setI2CBypass(false);
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
  static const float x_known[] = {0.0f, 0.211f, 0.710f}; // Voltage divider values
  static const float y_known[] = {0.0f, 3.26f, 11.0f};   // Actual voltages
  static const float offsets[] = {0.0f, -0.44f, -0.60f}; // Calibration offsets
  
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