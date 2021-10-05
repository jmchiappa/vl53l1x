/*!
 * @file Adafruit_VL53L1X.cpp
 *
 * @mainpage Adafruit VL53L1X time-of-flight sensor
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's VL53L1X driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit VL53L1X breakout: https://www.adafruit.com/product/3967
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Adafruit_VL53L1X.h"


Adafruit_VL53L1X::Adafruit_VL53L1X(uint8_t shutdown_pin, uint8_t irq_pin) 
  : VL53L1X(NULL, irq_pin) {
  _shutdown_pin = shutdown_pin;
  _irq_pin = irq_pin;
}

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
    @param  i2c_addr Optional I2C address the sensor can be found on. Default is 0x29
    @param debug Optional debug flag. If true, debug information will print out via Serial.print during setup. Defaults to false.
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool Adafruit_VL53L1X::begin(uint8_t i2c_addr, TwoWire *theWire, bool debug) {
  if (_shutdown_pin != -1) {
    pinMode(_shutdown_pin, OUTPUT);
    digitalWrite(_shutdown_pin, HIGH);
    digitalWrite(_shutdown_pin, LOW);
    delay(5);
    digitalWrite(_shutdown_pin, HIGH);
  }
  delay(5);

  theWire->begin();

  MyDevice.I2cHandle = theWire;
  dev_i2c = theWire;

  vl_status = InitSensor(i2c_addr * 2);
  if (vl_status != VL53L1X_ERROR_NONE) {
    return false;
  }

  if (sensorID() != 0xEACC) {
    return false;
  }

  /*
  Status = VL53L1_software_reset(pMyDevice);
  if (Status != VL53L1_ERROR_NONE) return false;

  // from Nucleo main.c
  Status = VL53L1_WaitDeviceBooted(pMyDevice);
  if (Status != VL53L1_ERROR_NONE) return false;

  if (debug) 
    Serial.println("DataInit");
  Status = VL53L1_DataInit(pMyDevice);         // Data initialization
  if (Status != VL53L1_ERROR_NONE) return false;

  if (debug) 
    Serial.println("StaticInit");
  Status = VL53L1_StaticInit(pMyDevice);
  if (Status != VL53L1_ERROR_NONE) return false;

  Status = VL53L1_GetDeviceInfo(pMyDevice, &DeviceInfo );
  if (Status != VL53L1_ERROR_NONE) return false;
  if (debug) {
    Serial.println( F( "VL53L1X Info:" ) );
    Serial.print( F( "Device Name: ")  ); Serial.print( DeviceInfo.Name );
    Serial.print( F( ", Type: " ) ); Serial.print( DeviceInfo.Type );
    Serial.print( F( ", ID: " ) ); Serial.println( DeviceInfo.ProductId );
    
    Serial.print( F( "Rev Major: " ) ); Serial.print( DeviceInfo.ProductRevisionMajor );
    Serial.print( F( ", Minor: " ) ); Serial.println( DeviceInfo.ProductRevisionMinor );
  }

  if (! SetDistanceMode(VL53L1_DISTANCEMODE_LONG)) return false;
  if (! SetMeasurementTimingBudgetMicroSeconds(50000)) return false;
  if (! SetInterMeasurementPeriodMilliSeconds(50)) return false;

  */
  return true;
}


uint16_t Adafruit_VL53L1X::sensorID(void) {
  uint16_t sensorID = 0;
  vl_status = VL53L1X_GetSensorId(&sensorID);
  return sensorID;
}

int16_t Adafruit_VL53L1X::distance(void) {
  uint16_t distance;
  
  vl_status = VL53L1X_GetDistance(&distance);
  if (vl_status != VL53L1X_ERROR_NONE) {
    return -1;
  }
  return (int16_t)distance;
}

bool Adafruit_VL53L1X::clearInterrupt(void) {
  vl_status = VL53L1X_ClearInterrupt();
  return (vl_status == VL53L1X_ERROR_NONE);
}

bool Adafruit_VL53L1X::setIntPolarity(bool polarity) {
  vl_status = VL53L1X_SetInterruptPolarity(polarity);
  return (vl_status == VL53L1X_ERROR_NONE);
}

bool Adafruit_VL53L1X::getIntPolarity(void) {
  uint8_t x = 0;
  vl_status = VL53L1X_GetInterruptPolarity(&x);
  return (bool)x;
}

bool Adafruit_VL53L1X::startRanging(void) {
  vl_status = VL53L1X_StartRanging();
  return (vl_status == VL53L1X_ERROR_NONE);
}

bool Adafruit_VL53L1X::stopRanging(void) {
  vl_status = VL53L1X_StopRanging();
  return (vl_status == VL53L1X_ERROR_NONE);
}

bool Adafruit_VL53L1X::dataReady(void) {
  uint8_t x = 0;
  vl_status = VL53L1X_CheckForDataReady(&x);
  return (bool)x;
}

bool Adafruit_VL53L1X::setTimingBudget(uint16_t ms) {
  vl_status = VL53L1X_SetTimingBudgetInMs(ms);
  return (vl_status == VL53L1X_ERROR_NONE);
}

uint16_t Adafruit_VL53L1X::getTimingBudget(void) {
  uint16_t ms = 0;

  vl_status = VL53L1X_GetTimingBudgetInMs(&ms);
  if (vl_status == VL53L1X_ERROR_NONE) {
    return ms;
  }
  return 0;
}


/*

}

boolean Adafruit_VL53L1X::SetDistanceMode(VL53L1_DistanceModes mode) {
  Status = VL53L1_SetDistanceMode(pMyDevice, mode );
  return (Status == VL53L1_ERROR_NONE);
}

boolean Adafruit_VL53L1X::GetRangingMeasurementData(VL53L1_RangingMeasurementData_t *ranging) {
  Status = VL53L1_GetRangingMeasurementData(pMyDevice, ranging);
  return (Status == VL53L1_ERROR_NONE);
}
*/
