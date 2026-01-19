/*!
 * @file Adafruit_BMP3XX.h
 *
 * Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Ladyada for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef __BMP3XX_H__
#define __BMP3XX_H__

#include "bmp3.h"

#include <Adafruit_I2CDevice.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_Sensor.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define BMP3XX_DEFAULT_ADDRESS (0x77) ///< The default I2C address
/*=========================================================================*/
#define BMP3XX_DEFAULT_SPIFREQ (1000000) ///< The default SPI Clock speed

#define SEALEVELPRESSURE_HPA (1013.25)

/** Adafruit_BMP3XX Class for both I2C and SPI usage.
 *  Wraps the Bosch library for Arduino usage
 */

class Adafruit_BMP3XX {
public:
  Adafruit_BMP3XX();

  bool begin_I2C(uint8_t addr = BMP3XX_DEFAULT_ADDRESS,
                 TwoWire *theWire = &Wire);
  bool begin_SPI(uint8_t cs_pin, SPIClass *theSPI = &SPI,
                 uint32_t frequency = BMP3XX_DEFAULT_SPIFREQ);
  bool begin_SPI(int8_t cs_pin, int8_t sck_pin, int8_t miso_pin,
                 int8_t mosi_pin, uint32_t frequency = BMP3XX_DEFAULT_SPIFREQ);
  
  uint8_t chipID(void);
  float readTemperature(void);
  float readPressure(void);
  float readAltitude(float seaLevel = SEALEVELPRESSURE_HPA);

  bool setTemperatureOversampling(uint8_t os);
  bool setPressureOversampling(uint8_t os);
  bool setIIRFilterCoeff(uint8_t fs);
  bool setOutputDataRate(uint8_t odr);

  void getEvent(sensors_event_t *temp, sensors_event_t *pressure, sensors_event_t *altitude);
  void fillTemperatureEvent(sensors_event_t *event, uint32_t timestamp);
  void fillPressureEvent(sensors_event_t *event, uint32_t timestamp);
  void fillAltitudeEvent(sensors_event_t *event, uint32_t timestamp);

  /// Perform a reading in blocking mode
  bool performReading(void);

  /// Temperature (Celsius) assigned after calling performReading()
  double temperature;
  /// Pressure (Pascals) assigned after calling performReading()
  double pressure;

private:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface

  struct bmp3_dev the_sensor;
  bool _init(void);

  bool _filterEnabled, _tempOSEnabled, _presOSEnabled, _ODREnabled;
};

#endif
