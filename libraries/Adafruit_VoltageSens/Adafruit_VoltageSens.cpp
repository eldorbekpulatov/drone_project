#include "Adafruit_VoltageSens.h"

float Adafruit_VoltageSens::_interpolate(float x1, float y1, float x2, float y2, float x, float offset) {
  // Handle edge cases where x is outside the defined range (optional, for extrapolation)
  if (x <= x1) return y1 + (x - x1) * (y2 - y1) / (x2 - x1); // Extrapolate using the first segment's slope
  if (x >= x2) return y2 + (x - x2) * (y2 - y1) / (x2 - x1); // Extrapolate using the last segment's slope

  // Linear interpolation formula
  return (y1 + (x - x1) * (y2 - y1) / (x2 - x1)) + offset;
}


void Adafruit_VoltageSens::_fillVoltageEvent(sensors_event_t *event, float voltage, uint32_t timestamp) {
  memset(event, 0, sizeof(sensors_event_t));
  event->version = 1;
  event->sensor_id = _adc_pin;
  event->type = SENSOR_TYPE_VOLTAGE;
  event->timestamp = timestamp;
  // set voltage value in the event (field exists in Adafruit Unified Sensor)
  event->voltage = voltage;
}


/**************************************************************************/
/*!
    @brief  Gets the voltage as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True if valid reading, false otherwise
*/
/**************************************************************************/
bool Adafruit_VoltageSens::getEvent(sensors_event_t *event) {
  // Validate configured data
  if (_adc_pin < 0 || !_x_known || !_y_known || _n_points < 2) {
    // Fill a zeroed/invalid event
    _fillVoltageEvent(event, 0.0f, (uint32_t)millis());

    return false;
  }

  int raw = analogRead(_adc_pin); // ADC reading (0-1023 for 10-bit 1V max)

  // Update readings array for averaging
  _sum -= _readings[_readingIndex]; // Subtract the oldest reading
  _readings[_readingIndex] = raw;   // Store the new reading
  _sum += raw;                      // Add the new reading to the sum
  _readingIndex = (_readingIndex + 1) % ADAFRUIT_VOLTAGESENS_MAX_SAMPLES; // Advance index
  
  if (!_fullSamples && _readingIndex == 0) {
    _fullSamples = true; // We have filled the buffer at least once
  }
  
  // calculate number of samples currently contributing
  int samples = _fullSamples ? ADAFRUIT_VOLTAGESENS_MAX_SAMPLES : _readingIndex;
  if (samples == 0) samples = 1; // avoid divide-by-zero on first call
  
  float averagedRaw = (float)_sum / (float)samples;
  
  // Convert averagedRaw ADC value to voltage
  float voltage = (averagedRaw / 1023.0f) * _maxVoltage;

  // Use voltage as the x value to interpolate against the provided tables.
  float x_to_interpolate = voltage;

  // find interval index
  size_t i = 0;
  while (i < (_n_points - 1) && _x_known[i + 1] < x_to_interpolate) { i++; }

  // perform interpolation between i and i+1 (handle end case)
  float interpolated_y;
  if (i >= _n_points - 1) {
    // extrapolate using last segment
    interpolated_y = _interpolate(_x_known[_n_points - 2], _y_known[_n_points - 2],
                                  _x_known[_n_points - 1], _y_known[_n_points - 1],
                                  x_to_interpolate, _offsett[_n_points - 1]);
  } else {
    interpolated_y = _interpolate(_x_known[i], _y_known[i],
                                  _x_known[i + 1], _y_known[i + 1],
                                  x_to_interpolate, _offsett[i]);
  }

  // Fill event with the interpolated value (interpreted as voltage/current/etc.)
  _fillVoltageEvent(event, interpolated_y, (uint32_t)millis());

  return true;
}


/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the voltage sensor
*/
/**************************************************************************/
void Adafruit_VoltageSens::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "VoltageSensor", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _adc_pin;
  sensor->type = SENSOR_TYPE_VOLTAGE;
  sensor->min_delay = 0;
  sensor->min_value = 0; /* volts */
  sensor->max_value = _maxVoltage;  /* volts */
  sensor->resolution = (sensor->max_value - sensor->min_value)/1024; /* 10-bit*/
}
