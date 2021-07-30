/* LED Blink, for Ladybug
 
   This example code is in the public domain.
*/
#include "STM32WB.h"
#include "RTC.h"
#include "BMA400.h"
#include "LC709204F.h"
#include "CCS811.h"
#include "LPS22HB.h"
#include "HDC2010.h"
#include "VEML6040.h"
#include "SFLASH.h"
#include "TimerMillis.h"

#define myButton     23
#define myLed         7 // blue led active HIGH

#define I2C_BUS    Wire1              // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

TimerMillis LoggerTimer;
volatile bool Logger_flag = false;
TimerMillis SensorTimer;
volatile bool Sensor_flag = false;

uint8_t mid;
uint16_t did;
uint16_t page_number = 0;     // set the page mumber for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

uint8_t seconds, minutes, hours, day, month, year;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

// Battery voltage monitor definitions
float VDDA, Temperature;
uint32_t UID[3] = {0, 0, 0};
volatile bool USBConnected = false; 
volatile bool SerialDebug = true;
volatile bool alarmFlag = false;

// Configure BMA400 accelerometer
#define BMA400_intPin1  20    // interrupt1 pin definitions, wake on motion
#define BMA400_intPin2  21    // interrupt2 pin definitions, sleep interrupt

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      SR_15_5Hz, SRW_25Hz, SR_50Hz, SR_100Hz, SR_200Hz, SR_400Hz, SR_800Hz 
      sleep_Mode, lowpower_Mode, normal_Mode, sleep_Mode
      osr0 (lowest power, lowest oversampling,lowest accuracy), osr1, osr2, osr3 (highest power, highest oversampling, highest accuracy)
      acc_filt1 (variable filter), acc_filt2 (fixed 100 Hz filter), acc_filt_lp (fixed 100 Hz filter, 1 Hz bandwidth)
*/ 
uint8_t Ascale = AFS_2G, SR = SR_200Hz, power_Mode = lowpower_Mode, OSR = osr0, acc_filter = acc_filt2;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 
float offset[3] = {0.0f, 0.0f, 0.0f}; // accel bias offsets
float delta[3] =  {0.0f, 0.0f, 0.0f}; // self-test results

// Logic flags to keep track of device states
volatile bool BMA400_wake_flag = false;
volatile bool BMA400_sleep_flag = false;
volatile bool InMotion = false;

BMA400 BMA400(&i2c_0); // instantiate BMA400 accel class


// Configure the LC709204F battery fuel gauge
#define LC709204F_intPin    9

uint32_t userID=0;
uint16_t batStatus = 0, RSOC = 0, timetoEmpty = 0, stateofHealth = 0, cellTemperature = 0, VBATraw;
float VBAT, ITE;
volatile bool LC709204F_alarm_flag = false;

LC709204F LC709204F(&i2c_0); // instantiate LC709204F class


// Configure CCS811 eCO2 and TVoC sensor
#define CCS811_intPin   5
#define CCS811_wakePin 11

/* Specify CCS811 sensor parameters
 *  Choices are   dt_idle , dt_1sec, dt_10sec, dt_60sec
 */
uint8_t AQRate = dt_60sec;  // set the sample rate
uint8_t rawData[8] = {0, 0, 0, 0, 0, 0, 0, 0};  // array to hold the raw data
uint16_t eCO2 = 0, TVOC = 0;
uint8_t Current = 0;
float Voltage = 0.0f;
int16_t compHumidity = 0, compTemp = 0;

volatile bool CCS811_intFlag  = true; // boolean flag for interrupt

CCS811 CCS811(&i2c_0); // instantiate CCS811 class


// Configure LPS22HB barometer
#define LPS22HB_intPin A1

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_1Hz;     // set pressure amd temperature output data rate
int32_t rawPressure = 0;
float LPSTemperature, LPSPressure, LPSAltitude;
uint8_t LPS22HBstatus, LPSintSource;
volatile bool LPS22HB_intFlag = true;

LPS22HB LPS22HB(&i2c_0);  // Instantiate LPS22HB barometer


// Configure HCD2010 humidity/temperature sensor
#define HDC2010_intPin   A0

// Choices are:
// freq = ForceMode, Freq_120s, Freq_60s, Freq_10s, Freq_5s, Freq_1s, Freq_0_5s, Freq_0_2s
// tres = TRES_14bit, TRES_11bit, TRES_9bit
// hres = HRES_14bit, HRES_11bit, HRES_9bit
uint8_t freq = Freq_10s, tres = TRES_14bit, hres = HRES_14bit;

float HDCTemperature = 0.0f, HDCHumidity = 0.0f;
uint8_t HDCStatus;
volatile bool HDC2010_intFlag = true;

HDC2010 HDC2010(&i2c_0); // Instantiate HDC2010 Humidity sensor


// Specify VEML6040 Integration time
/*Choices are:
 IT_40  40 ms, IT_80  80 ms, IT_160  160 ms, IT_320  320 ms, IT_640  640 ms, IT_1280  1280 ms*/
uint8_t IT = IT_40;  // integration time variable
uint8_t ITime = 40;  // integration time in milliseconds
uint16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (1 << IT) ); // ambient light sensitivity increases with integration time
float ambientLight;

VEML6040 VEML6040(&i2c_0);


void setup() 
{
  if(SerialDebug) Serial.begin(115200); // Debug output on Serial
//  while (!Serial) { }
  if(SerialDebug) Serial.println("Serial enabled!");

  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led on
  
  pinMode(BMA400_intPin1, INPUT);     // define BMA400 wake and sleep interrupt pins as L082 inputs
  pinMode(BMA400_intPin2, INPUT);
  pinMode(LC709204F_intPin, INPUT);   // set up  battery fuel gauge interrupt
  pinMode(CCS811_wakePin, OUTPUT);    // CCS811 wake pin active LOW
  pinMode(CCS811_intPin, INPUT);      // active LOW
  pinMode(HDC2010_intPin, INPUT);     // active HIGH
  pinMode(LPS22HB_intPin, INPUT);     // active HIGH

  STM32WB.getUID(UID);
  if(SerialDebug) {Serial.print("STM32L4 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);} 

   /* initialize wire bus */
  I2C_BUS.begin();                    // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);           // I2C frequency at 400 kHz
  delay(100);

  Serial.println("Scan for I2C devices:");
  digitalWrite(CCS811_wakePin, LOW);  // set LOW to enable the CCS811 air quality sensor
  i2c_0.I2Cscan();                    // should detect BMA400 at 0x14 and LPS22HB at 0x5C, etc a 
  digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor
  delay(100);
  
  /* Check internal STM32WB and battery power configuration */
  VDDA = STM32WB.readVDDA();
  Temperature = STM32WB.readTemperature();
  
  // Internal STM32WB functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
  Serial.println(" "); 

  // Read the BMA400 Chip ID register, this is a good test of communication
  Serial.println("BMA400 accelerometer...");
  byte BMA400_ID = BMA400.getChipID();  // Read CHIP_ID register for BMA400
  Serial.print("BMA400 "); Serial.print("I AM "); Serial.print(BMA400_ID, HEX); Serial.print(" I should be "); Serial.println(0x90, HEX);
  Serial.println(" ");
  delay(100);  

  // Read the LC709204F Chip ID register 
  Serial.println("LC709204F LiPo battery fuel gauge...");
  uint16_t LC709204_ID = LC709204F.getChipID();  // Read USER_ID register for LC709204F
  Serial.print("LC709204F "); Serial.print("I AM 0x"); Serial.print(LC709204_ID, HEX); Serial.print(" I should be "); Serial.println(0x1001, HEX);
  Serial.println(" ");
  delay(100); 

  // Read the WHO_AM_I register of the CCS811 this is a good test of communication
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  byte CCS811_ChipID = CCS811.getChipID();
  digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor
  Serial.print("CCS811 "); Serial.print("I AM "); Serial.print(CCS811_ChipID, HEX); Serial.print(" I should be "); Serial.println(0x81, HEX);
  Serial.println(" ");
  delay(100); 

  Serial.println("LPS22HB barometer...");
  uint8_t LPS22HB_chipID = LPS22HB.getChipID();
  Serial.print("LPS22HB "); Serial.print("I AM "); Serial.print(LPS22HB_chipID, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("HDC2010 humidity sensor...");
  uint16_t HDC2010_devID = HDC2010.getDevID(HDC2010_0_ADDRESS);
  Serial.print("DeviceID = 0x0"); Serial.print(HDC2010_devID, HEX); Serial.println(". Should be 0x07D0");

  uint16_t HDC2010_manuID = HDC2010.getManuID(HDC2010_0_ADDRESS);
  Serial.print("Manufacturer's ID = 0x"); Serial.print(HDC2010_manuID, HEX); Serial.println(". Should be 0x5449");
  delay(1000);

  if(BMA400_ID == 0x90 && LC709204_ID == 0x1001 && CCS811_ChipID == 0x81 && LPS22HB_chipID == 0xB1 && HDC2010_devID == 0x07D0) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("BMA400, LC709204F, CCS811, LPS22HB, HDC2010 are online..."); Serial.println(" ");
   
   aRes = BMA400.getAres(Ascale);                                       // get sensor resolutions, only need to do this once
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);      
   BMA400.selfTestBMA400(delta);                                        // perform sensor self test
   Serial.print("x-axis self test = "); Serial.print(delta[0], 1); Serial.println("mg, should be > 2000 mg");
   Serial.print("y-axis self test = "); Serial.print(delta[1], 1); Serial.println("mg, should be > 1800 mg");
   Serial.print("z-axis self test = "); Serial.print(delta[2], 1); Serial.println("mg, should be > 800 mg");
   BMA400.resetBMA400();                                                // software reset before initialization
   delay(100);                                                          // give some time to read the screen

   // Calibrate the accelerometer
   Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   BMA400.CompensationBMA400(Ascale, SR, normal_Mode, OSR, acc_filter, offset); // quickly estimate offset bias in normal mode
   Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
   Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
   Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");
   // End calibration

   BMA400.initBMA400(Ascale, SR, power_Mode, OSR, acc_filter);          // Initialize sensor in desired mode for application 

   LC709204F.init();
   batStatus = LC709204F.getStatus();
   if(batStatus & 0x0080) Serial.println("Fuel gauge initialized!");  
   if(batStatus & 0x0040) Serial.println("Fuel gauge discharging!");  
   stateofHealth = LC709204F.stateofHealth();
   Serial.print("Battery state of Health = "); Serial.println(stateofHealth, HEX);
   delay(100);   

   // initialize CCS811 and check version and status
   digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
   CCS811.CCS811init(AQRate);
   digitalWrite(CCS811_wakePin, HIGH); // set HIGH to disable the CCS811 air quality sensor

   LPS22HB.reset();
   LPS22HB.Init(PODR);  // Initialize LPS22HB barometer
   delay(100);
  
   HDC2010.reset(HDC2010_0_ADDRESS);

   // Configure HCD2010 for auto measurement mode if freq not ForceMode
   // else measurement performed only once each time init is called
   HDC2010.init(HDC2010_0_ADDRESS, hres, tres, freq); 

   digitalWrite(myLed, LOW);  // when sensors successfully configured, turn off led
  }
   else 
  {
   if(BMA400_ID != 0x90)       Serial.println(" BMA400 not functioning!");
   if(LC709204_ID != 0x1001)   Serial.println(" LC709204F not functioning!");
   if(CCS811_ChipID != 0x81)   Serial.println(" CCS811 not functioning!"); 
   if(LPS22HB_chipID != 0xB1)  Serial.println(" LPS22HB2 not functioning!");   
   if(HDC2010_devID != 0x07D0) Serial.println(" HDC20102 not functioning!");
  }

  VEML6040.enableVEML6040(IT); // initalize VEML6040 sensor
  delay(150); 

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
  Serial.println(" ");

  // Erase memory before data logging
//  uint32_t address;
//  for (address = 0; address < 4096 * 1024; address += SFLASH.blockSize()) {
//        SFLASH.erase(address);
//  }
  
  // set alarm to update the RTC periodically
  RTC.enableAlarm(RTC_MATCH_ANY); // alarm once a second
  RTC.attachInterrupt(alarmMatch);

//  LoggerTimer.start(callbackLogger, 10000, 10000);    //  10 second period, delayed 10 seconds ~ 30 days of logging
  LoggerTimer.start(callbackLogger, 60000, 120000);      //  60 second period, delayed 120 seconds ~ 180 days of logging

  SensorTimer.start(callbackSensor, 30000, 30000);      //  30 second period, delayed 30 seconds 

  attachInterrupt(BMA400_intPin1,   myinthandler1, RISING);     // attach wake-up   interrupt for INT1 pin output of BMA400
  attachInterrupt(BMA400_intPin2,   myinthandler2, RISING);     // attach no-motion interrupt for INT2 pin output of BMA400 
  attachInterrupt(LC709204F_intPin, myinthandler3, FALLING);    // attach interrupt for ALARM pin output of LC709204F 
  attachInterrupt(CCS811_intPin,    myinthandler4, FALLING);    // enable CCS811 interrupt 
  attachInterrupt(HDC2010_intPin,   myinthandler5, RISING);     // define interrupt for INT pin output of HDC2010
  attachInterrupt(LPS22HB_intPin,   myinthandler6, RISING);     // define interrupt for INT pin output of LPS22HB

  // read interrupt status register(s) to unlatch interrupts before entering main loop
  HDCStatus  = HDC2010.getIntStatus(HDC2010_0_ADDRESS);
  LPSintSource = LPS22HB.intSource();
  BMA400.getStatus(); // read status of interrupts to clear
  LC709204F.clearStatus();
  digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
  CCS811.readCCS811Data(rawData);    // read CCS811 data to clear interrupt
  digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 
} // end of setup


void loop() 
{
  /* BMA400 sleep/wake detect*/
  if(BMA400_wake_flag)
  {
   BMA400_wake_flag = false; // clear the wake flag
   InMotion = true;          // set motion state latch
   Serial.println("** BMA400 is awake! **");
   BMA400.activateNoMotionInterrupt();  
   attachInterrupt(BMA400_intPin2, myinthandler2, RISING);  // attach no-motion interrupt for INT2 pin output of BMA400 

   digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);
  }

  if(BMA400_sleep_flag)
  {
   BMA400_sleep_flag = false;            // clear the sleep flag
   InMotion = false;                     // set motion state latch
   Serial.println("** BMA400 is asleep! **");
   detachInterrupt(BMA400_intPin2);      // Detach the BMA400 "Go to sleep" interrupt so it doesn't spuriously wake the STM32L4
   BMA400.deactivateNoMotionInterrupt(); // disable no-motion interrupt to save power 

   digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);
  }/* end of sleep/wake detect */


  // LC709204F alarm handling
  if(LC709204F_alarm_flag)
  {
     LC709204F_alarm_flag = false;

   Serial.println("** LC709204F alarm! **");
   batStatus = LC709204F.getStatus();
   LC709204F.clearStatus();
   if(batStatus & 0x0800) Serial.println("Low Cell Voltage Alarm!");  
   if(batStatus & 0x0200) Serial.println("Low RSOC Alarm!");  
   if(batStatus & 0x1000) Serial.println("High temperature Alarm!");  
   stateofHealth = LC709204F.stateofHealth();
   Serial.print("Battery state of Health = "); Serial.println(stateofHealth, HEX);
  }/* end of battery alarm */


    // CCS811 data
    // If intPin goes LOW, all data registers have new data
    if(CCS811_intFlag == true) {  // On interrupt, read data
       CCS811_intFlag = false;  // reset newData flag
     
    digitalWrite(CCS811_wakePin, LOW); // set LOW to enable the CCS811 air quality sensor
    CCS811.readCCS811Data(rawData);
    CCS811.compensateCCS811(compHumidity, compTemp); // compensate CCS811 using BME280 humidity and temperature
    digitalWrite(CCS811_wakePin, HIGH); // set LOW to enable the CCS811 air quality sensor 

    eCO2 = (uint16_t) ((uint16_t) rawData[0] << 8 | rawData[1]);
    TVOC = (uint16_t) ((uint16_t) rawData[2] << 8 | rawData[3]);
    Current = (rawData[6] & 0xFC) >> 2;
    Voltage = (float) ((uint16_t) ((((uint16_t)rawData[6] & 0x02) << 8) | rawData[7])) * (1.65f/1023.0f); 
    
    Serial.println("CCS811:");
    Serial.print("Eq CO2 in ppm = "); Serial.println(eCO2);
    Serial.print("TVOC in ppb = "); Serial.println(TVOC);
    Serial.print("Sensor current (uA) = "); Serial.println(Current);
    Serial.print("Sensor voltage (V) = "); Serial.println(Voltage, 2);  
    Serial.println(" ");
  }/* end of CCS811 interrupt handling */


    if(HDC2010_intFlag)
  {
       HDC2010_intFlag = false; // reset HDC2010 interrupt flag
    
    HDCTemperature = HDC2010.getTemperature(HDC2010_0_ADDRESS); // float degrees C
    HDCHumidity = HDC2010.getHumidity(HDC2010_0_ADDRESS);       // float %rel humidity
    compTemp = 100 * HDCTemperature;  // range from 0 to 10000
    compHumidity = 100 * HDCHumidity; // range from -4000 to 16500
 
    if(SerialDebug) {
      Serial.print("HDC2010 Temperature is "); Serial.print(HDCTemperature, 2); Serial.println(" degrees C");
    }
    if(SerialDebug) {
      Serial.print("HDC2010 Humidity is "); Serial.print(HDCHumidity, 2); Serial.println(" %RH");
    }
      Serial.println(" ");
  } /* end of HDC2010 interrupt handling*/


  if(LPS22HB_intFlag)
  {
     LPS22HB_intFlag = false; // reset LPS22HB interrupt flag

   rawPressure = LPS22HB.readAltimeterPressure();
   LPSPressure = (float) rawPressure/4096.0f;
   LPSTemperature = (float) LPS22HB.readAltimeterTemperature()/100.0f; 
   LPSAltitude = 145366.45f*(1.0f - pow((LPSPressure/1013.25f), 0.190284f)); 

   if(SerialDebug) {
     Serial.print("Baro temperature = "); Serial.print(LPSTemperature, 2); Serial.print(" C"); // temperature in degrees Celsius  
//     Serial.print("Altimeter temperature = "); Serial.print(9.0f*LPSTemperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
     Serial.println(" ");
     Serial.print("Baro pressure = "); Serial.print(LPSPressure, 2);  Serial.print(" mbar");// pressure in millibar
     Serial.println(" ");     
     Serial.print("Altitude = "); Serial.print(LPSAltitude, 2); Serial.println(" ft");
     Serial.println(" ");
     }
  } /* end of LPS22HB interrupt handling */


    if (Sensor_flag) { // update data for sensors with no interrupt
        Sensor_flag = false;

    Temperature = STM32WB.readTemperature();
    VDDA = STM32WB.readVDDA();
    // Battery fuel gauge measurements
//    LC709204F.operate();
    // Set cell temperature as MCU temperature in units of 0.1 K
    LC709204F.setTemperature(2732 +  ((int16_t) (10.0f * Temperature)) );
//    Serial.println(2732 +  ((int16_t) (10.0f * Temperature)) , HEX);
//    cellTemperature = LC709204F.getTemperature(); // write only in I2C mode

    VBATraw = LC709204F.getCellVoltage();
    VBAT = ((float) VBATraw )/1000.0f;
    RSOC = LC709204F.getRSOC(); // range from 0x0000 to 0x0064 so LSByte only needed
    timetoEmpty = LC709204F.timetoEmpty();
    ITE = ((float) LC709204F.getITE()) / 10.0f;
//    LC709204F.sleep(); // sleep current 1.3 uA, operate current ~2 uA
     
   if(SerialDebug) {
      Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
      Serial.print("VBAT = "); Serial.print(VBAT, 3); Serial.println(" V");
      Serial.print("RSOC = "); Serial.print(RSOC); Serial.println(" %");
      Serial.print("Time to Empty = "); Serial.print(timetoEmpty); Serial.println(" min");
      Serial.print("Indicator to Empty = "); Serial.print(ITE, 1); Serial.println(" %");
      Serial.print("STM32L4 MCU Temperature = "); Serial.println(Temperature, 2);
//      Serial.print("Cell Temperature = 0x"); Serial.println(cellTemperature, HEX);
      Serial.println(" ");
    }

   /* VEML6040 Data */
  VEML6040.enableVEML6040(IT); // enable VEML6040 sensor
  delay(ITime + 5);  // wait for integration of light sensor data
  VEML6040.getRGBWdata(RGBWData); // read light sensor data
  VEML6040.disableVEML6040(IT); // disable VEML6040 sensor
 
  Serial.print("Red raw counts = ");   Serial.println(RGBWData[0]);
  Serial.print("Green raw counts = "); Serial.println(RGBWData[1]);
  Serial.print("Blue raw counts = ");  Serial.println(RGBWData[2]);
  Serial.print("White raw counts = "); Serial.println(RGBWData[3]);
  Serial.print("Inferred IR raw counts = "); Serial.println(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]);
  Serial.println("  ");
 
  Serial.print("Red   light power density = "); Serial.print((float)RGBWData[0]/96.0f, 2); Serial.println(" microWatt/cm^2");
  Serial.print("Green light power density = "); Serial.print((float)RGBWData[1]/74.0f, 2); Serial.println(" microWatt/cm^2");
  Serial.print("Blue  light power density = "); Serial.print((float)RGBWData[2]/56.0f, 2); Serial.println(" microWatt/cm^2");
  Serial.println("  ");

  ambientLight = (float)RGBWData[1]*GSensitivity;
  Serial.print("Ambient light intensity = "); Serial.print(ambientLight, 2); Serial.println(" lux");
  Serial.println("  ");

  // Empirical estimation of the correlated color temperature CCT:
  // see https://www.vishay.com/docs/84331/designingveml6040.pdf
  float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
  float CCT = 4278.6f*powf(temp, -1.2455f) + 0.5f;

  Serial.print("Correlated Color Temperature = "); Serial.print(CCT); Serial.println(" Kelvin");
  Serial.println("  ");
  /* end of VEML6040 data handling */
    } /* end of non-interrupt sensor data

  
  /*RTC*/
  if (alarmFlag) { // update RTC output at the alarm
      alarmFlag = false;
    
    if(SerialDebug && InMotion) {
      
    BMA400.readBMA400AccelData(accelCount); // get 12-bit signed accel data

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - offset[1];   
    az = (float)accelCount[2]*aRes - offset[2]; 
     
    Serial.println(" ");
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.println(" ");

    tempCount = BMA400.readBMA400TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        
    }

  Serial.println("RTC:");
  Day = RTC.getDay();
  Month = RTC.getMonth();
  Year = RTC.getYear();
  Seconds = RTC.getSeconds();
  Minutes = RTC.getMinutes();
  Hours   = RTC.getHours();     
  if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
  Serial.print(":"); 
  if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
  Serial.print(":"); 
  if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.print(Seconds);  
  Serial.println(" ");
  Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
  Serial.println(" ");

  digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);
 } // end of RTC alarm section


 /* Log some data to the QSPI flash */
  if(Logger_flag) {
     Logger_flag = false;
  
    // Highest page number is 0x7FFF = 32767 for 64 Mbit flash
    // store some data to the SPI flash
    uint8_t bps = 32; // bytes per sector such that 256 bytes per page= sectors per page x bps = 8 x 32
      if(sector_number < 8 && page_number < 0x7FFF) {
      flashPage[sector_number*bps + 0]  = InMotion; // accelerometer awoke on motion?
      flashPage[sector_number*bps + 1]  = rawData[0];                    // eCO2 MSB
      flashPage[sector_number*bps + 2]  = rawData[1];                    // eCO2 LSB
      flashPage[sector_number*bps + 3]  = rawData[2];                    // TVOC MSB
      flashPage[sector_number*bps + 4]  = rawData[3];                    // TVOC LSB
      flashPage[sector_number*bps + 5] = (compTemp & 0xFF00) >> 8;      // HDC2010 temperature
      flashPage[sector_number*bps + 6] = (compTemp & 0x00FF); 
      flashPage[sector_number*bps + 7] = (compHumidity & 0xFF00) >> 8;  // HDC2010 humidity
      flashPage[sector_number*bps + 8] = (compHumidity & 0x00FF);
      flashPage[sector_number*bps + 9] = (rawPressure & 0x00FF0000) >> 16; //LPS22HB raw Pressure
      flashPage[sector_number*bps + 10] = (rawPressure & 0x0000FF00) >> 8; 
      flashPage[sector_number*bps + 11] = (rawPressure & 0x000000FF);
      flashPage[sector_number*bps + 12] = Seconds;                       // RTC time and date
      flashPage[sector_number*bps + 13] = Minutes;
      flashPage[sector_number*bps + 14] = Hours;
      flashPage[sector_number*bps + 15] = Day;
      flashPage[sector_number*bps + 16] = Month;
      flashPage[sector_number*bps + 17] = Year;
      flashPage[sector_number*bps + 18] = (RGBWData[0] & 0xFF00) >> 8;   // VEML6040 RGBW data
      flashPage[sector_number*bps + 19] = (RGBWData[0] & 0x00FF);
      flashPage[sector_number*bps + 20] = (RGBWData[1] & 0xFF00) >> 8;
      flashPage[sector_number*bps + 21] = (RGBWData[1] & 0x00FF);
      flashPage[sector_number*bps + 22] = (RGBWData[2] & 0xFF00) >> 8;
      flashPage[sector_number*bps + 23] = (RGBWData[2] & 0x00FF);
      flashPage[sector_number*bps + 24] = (RGBWData[3] & 0xFF00) >> 8;
      flashPage[sector_number*bps + 25] = (RGBWData[3] & 0x00FF);
      flashPage[sector_number*bps + 26] = (VBATraw & 0xFF00) >> 8; // VBATraw from fuel gauge
      flashPage[sector_number*bps + 27] =  VBATraw & 0x00FF;
      flashPage[sector_number*bps + 28] =  RSOC & 0x00FF;          // RSOC from fuel gauge
      sector_number++;
      }
       
      if(sector_number == 8 && page_number < 0x7FFF)
      {
       SFLASH.program(page_number * 256, flashPage, 256);  // write next 256-byte page
       Serial.print("Wrote flash page: "); Serial.println(page_number);
       digitalWrite(myLed, HIGH); delay(1); digitalWrite(myLed, LOW);
       sector_number = 0;
       page_number ++;
      }  
      else if(page_number == 0x7FFF) 
      {
       Serial.println("Reached last page of SPI flash!"); Serial.println("Data logging stopped!");
      }
  } /* End of QSPI Flash data logging section */
  

  STM32WB.stop();        // Enter STOP or SLEEP mode and wait for an interrupt

} // end of main loop


/* Useful functions */
void myinthandler1()
{
  BMA400_wake_flag = true;
  STM32WB.wakeup(); 
}


void myinthandler2()
{
  BMA400_sleep_flag = true;
  STM32WB.wakeup();
}


void myinthandler3()
{
  LC709204F_alarm_flag = true;
  STM32WB.wakeup();
}


  void myinthandler4()
{
  CCS811_intFlag = true;
  STM32WB.wakeup();
}


void myinthandler5()
{
  HDC2010_intFlag = true;
  STM32WB.wakeup();
}


void myinthandler6()
{
  LPS22HB_intFlag = true;
  STM32WB.wakeup();
}


void callbackLogger()
{
  Logger_flag = true; 
  STM32WB.wakeup();
}


void callbackSensor()
{
  Sensor_flag = true; 
  STM32WB.wakeup();
}


void alarmMatch()
{
  alarmFlag = true;
  STM32WB.wakeup();
}
