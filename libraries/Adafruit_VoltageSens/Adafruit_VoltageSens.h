/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software< /span>
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Update by K. Townsend (Adafruit Industries) for lighter typedefs, and
 * extended sensor support to include color, voltage and current */

#ifndef _ADAFRUIT_VOLTSAGE_SENS_H
#define _ADAFRUIT_VOLTSAGE_SENS_H


#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ADAFRUIT_VOLTAGESENS_MAX_SAMPLES 8  // compile-time max samples; change as needed

/** Adafruit Unified Sensor interface for gyro component of MPU6050 */
class Adafruit_VoltageSens : public Adafruit_Sensor {
public:
  // default constructor (safe no-arg) so existing code that does `Adafruit_VoltageSens volt;` compiles
  Adafruit_VoltageSens()
    : _adc_pin(-1), _maxVoltage(0.0f), _x_known(nullptr), _y_known(nullptr), _offsett(nullptr),
      _n_points(0), _readingIndex(0), _sum(0), _fullSamples(false)
  {
    // ensure readings are zeroed
    for (size_t i = 0; i < ADAFRUIT_VOLTAGESENS_MAX_SAMPLES; ++i) _readings[i] = 0;
  }
  
  // main constructor: pass pointers to arrays and element count
  Adafruit_VoltageSens(int adc_pin, float maxVoltage, const float *x_known, const float *y_known, 
    const float *offset, size_t n_points)
    : _adc_pin(adc_pin), _maxVoltage(maxVoltage), 
      _x_known(x_known), _y_known(y_known), _offsett(offset),_n_points(n_points),
      _readingIndex(0), _sum(0), _fullSamples(false)
    {
      // zero the buffer
      for (size_t i = 0; i < ADAFRUIT_VOLTAGESENS_MAX_SAMPLES; ++i) _readings[i] = 0;
    }
  
  ~Adafruit_VoltageSens() {}

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  int _adc_pin;                    // ADC pin number
  float _maxVoltage;               // Max voltage of the ADC reference

  const float *_x_known;           // Known x-values (pointer to externally-owned array)
  const float *_y_known;           // Known y-values (pointer to externally-owned array)
  const float *_offsett;           // calibration offsets (pointer to externally-owned array)
  size_t _n_points;                // number of points in the x/y/offset arrays

  int _readings[ADAFRUIT_VOLTAGESENS_MAX_SAMPLES];  // fixed-size buffer (no heap)
  int _readingIndex;               // Index for the readings array
  int _sum;                        // Sum of the readings
  bool _fullSamples;               // flag to check if we have taken enough samples
  
  float _interpolate(float x1, float y1, float x2, float y2, float x, float offset); // utility
  void _fillVoltageEvent(sensors_event_t *gyro, float voltage, uint32_t timestamp); // utility
};


#endif
