/* readSPIFlash_Rodan.v01a
 *  
2021 Copyright Tlera Corporation 

July 30, 2021

Sketch to read the QSPI Flash on the Rodan.v01aa, reconstruct the sensor data, and output CMS-compatible data for plotting.
 */

#include "SFLASH.h"
#include "STM32WB.h"

// Highest page number is 0x7FFF = 32767 for 64 Mbit flash
uint16_t max_page_number = 0x7FFF;
unsigned char flashPage[256];
uint16_t page_number = 0;
uint8_t sector_number = 0;
uint8_t mid;
uint16_t did;

int16_t compHumidity = 0, compTemp = 0;
int32_t rawPress = 0;
float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280
uint8_t Seconds, Minutes, Hours, Day, Month, Year;
uint16_t VBATraw;
float VDDA, VBAT;
uint16_t eCO2 = 0, TVOC = 0;
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
uint8_t RSOC;
/*Choices are:
 IT_40  40 ms, IT_80 80 ms, IT_160  160 ms, IT_320  320 ms, IT_640 640 ms, IT_1280 1280 ms*/
uint8_t IT = 0;  // integration time variable IT_40
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT)); // ambient light sensitivity increases with integration time
uint8_t InMotion =0;

void setup(void)
{ 
  Serial.begin(115200);
  while (!Serial) { }
  Serial.println("Serial enabled!");

  // Test QSPI flash memory
  Serial.println("QSPI Flash Check");
  SFLASH.begin();
  SFLASH.identify(mid, did);
  Serial.print("MID = ");       Serial.println(mid, HEX); 
  Serial.print("DID = ");       Serial.println(did, HEX); 
  Serial.print("CAPACITY = ");  Serial.println(SFLASH.capacity());
  Serial.print("BLOCKSIZE = "); Serial.println(SFLASH.blockSize());
  Serial.print("PAGESIZE = ");  Serial.println(SFLASH.pageSize());
  Serial.print("LENGTH = ");    Serial.println(SFLASH.length()); Serial.println(" ");
 
  uint8_t bps = 32; // bytes per sector such that 256 bytes per page= sectors per page x bps = 8 x 32

  // read the RODAN SPI flash
  for(page_number = 0; page_number < 100; page_number++)  { // change the page number limit to correspond to number of pages logged

//  Serial.print("Read Page 0x"); Serial.println(page_number, HEX);
  SFLASH.read(page_number * 256, flashPage, sizeof(flashPage));
      
   for(sector_number = 0; sector_number < 8; sector_number++) {

    InMotion = flashPage[sector_number*bps + 0];
     
    compTemp = ((int16_t) flashPage[sector_number*bps + 5] << 8) | flashPage[sector_number*bps + 6];
    compHumidity = ((int16_t) flashPage[sector_number*bps + 7] << 8) |  flashPage[sector_number*bps + 8];
    rawPress = ((int32_t)flashPage[sector_number*bps + 9] << 16) |  ((int32_t)flashPage[sector_number*bps + 10] << 8) | flashPage[sector_number*bps + 11];

    eCO2 = (uint16_t) ((uint16_t) flashPage[sector_number*bps + 1] << 8 | flashPage[sector_number*bps + 2]);
    TVOC = (uint16_t) ((uint16_t) flashPage[sector_number*bps + 3] << 8 | flashPage[sector_number*bps + 4]);

    Seconds = flashPage[sector_number*bps + 12];
    Minutes = flashPage[sector_number*bps + 13];
    Hours =   flashPage[sector_number*bps + 14];
    Day =     flashPage[sector_number*bps + 15];
    Month =   flashPage[sector_number*bps + 16];
    Year =    flashPage[sector_number*bps + 17];

    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    pressure = (float) rawPress/4096.0f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
   
    humidity = (float)compHumidity/100.0f; // Humidity in %RH

    RGBWData[0] = ((uint16_t) flashPage[sector_number*bps + 18] << 8) |  flashPage[sector_number*bps + 19];
    RGBWData[1] = ((uint16_t) flashPage[sector_number*bps + 20] << 8) |  flashPage[sector_number*bps + 21];
    RGBWData[2] = ((uint16_t) flashPage[sector_number*bps + 22] << 8) |  flashPage[sector_number*bps + 23];
    RGBWData[3] = ((uint16_t) flashPage[sector_number*bps + 24] << 8) |  flashPage[sector_number*bps + 25];

    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
      float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
      float CCT = 4278.6f*powf(temp, - 1.2455f) + 0.5f;

      VBATraw = ((uint16_t) flashPage[sector_number*bps + 26] << 8) |  flashPage[sector_number*bps + 27];
      VBAT =  (float) VBATraw / 1000.0f;

      RSOC = flashPage[sector_number*bps + 28];

    // Output for spreadsheet analysis
    if(Month < 10) {Serial.print("0"); Serial.print(Month);} else Serial.print(Month);
    Serial.print("/");Serial.print(Day); Serial.print("/");Serial.print(Year); Serial.print(" ");
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.print(Seconds);} else Serial.print(Seconds); Serial.print(",");      
    Serial.print(pressure, 2); Serial.print(","); Serial.print(temperature_C, 2); Serial.print(",");Serial.print(temperature_F, 2); Serial.print(",");
    Serial.print(altitude, 2); Serial.print(","); Serial.print(humidity, 1); Serial.print(","); 
    Serial.print(eCO2); Serial.print(","); Serial.print(TVOC); Serial.print(","); 
    Serial.print((float)RGBWData[0]/96.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[1]/74.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[2]/56.0f, 2); Serial.print(",");
    Serial.print((float)RGBWData[1]*GSensitivity, 2); Serial.print(",");
    Serial.print(VBAT,2); Serial.print(","); Serial.print(RSOC); Serial.print(",");
    Serial.println(InMotion);  
    }
  
  }


}

void loop(void)
{
}
