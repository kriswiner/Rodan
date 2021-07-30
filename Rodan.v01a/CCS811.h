/* 06/16/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 *  
 *  The AMS CCS811 is an air quality sensor that provides equivalent CO2 and volatile organic measurements from direct
 *  I2C register reads as well as current and voltage (effective resistance of the sensing element). Gas sensors, including 
 *  this MEMs gas sensor in the CCS811 measure resistance of a substrate that changes when exposed to inert gasses and 
 *  volatile organic compounds. Changed in concentration vary exponentially with the changes in resistance. The CCS811
 *  has an embedded ASIC calibrated against most common indoor pollutants that returns a good estimate of
 *  equivalent CO2 concentration in parts per million (400 - 8192 range) and volatile organic componds in parts per billion (0 - 1187).
 *  The sensor is quite sensitive to breath and other human emissions.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#ifndef CCS811_h
#define CCS811_h

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"

/* CCS811 Registers
http://www.mouser.com/ds/2/588/CCS811_DS000459_3-00-1098798.pdf
*/
#define CCS811_STATUS             0x00
#define CCS811_MEAS_MODE          0x01
#define CCS811_ALG_RESULT_DATA    0x02
#define CCS811_RAW_DATA           0x03
#define CCS811_ENV_DATA           0x05
#define CCS811_NTC                0x06
#define CCS811_THRESHOLDS         0x10
#define CCS811_BASELINE           0x11
#define CCS811_HW_ID              0x20  // WHO_AM_I should be 0x81
#define CCS811_ID                 0x20  // WHO_AM_I should be 0x1X
#define CCS811_HW_VERSION         0x21  
#define CCS811_FW_BOOT_VERSION    0x23
#define CCS811_FW_APP_VERSION     0x24
#define CCS811_ERROR_ID           0xE0
#define CCS811_APP_START          0xF4
#define CCS811_SW_RESET           0xFF

#define CCS811_ADDRESS            0x5A   // Address of the CCS811 Air Quality Sensor

#define  dt_idle  0x00
#define  dt_1sec  0x01
#define  dt_10sec 0x02
#define  dt_60sec 0x03

class CCS811
{
  public: 
  CCS811(I2Cdev* i2c_bus);
  void checkCCS811Status();
  void CCS811init(uint8_t AQRate);
  uint8_t getChipID();
  void compensateCCS811(int16_t compHumidity, int16_t compTemp);
  void readCCS811Data(uint8_t * destination);
  
  private:
  I2Cdev* _i2c_bus;
};


#endif
