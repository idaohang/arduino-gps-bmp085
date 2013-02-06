/*************************************************** 
  This is a library for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution


 *****************************************************
   Modified on: 5 févr. 2013
       Author: neuf

 Heavily altered by me :
     - more straightforward computation (my aim is altitude, not temp or pressure)
     - Use of the EOC pin (get rid of all the delays...)
     - Sequencing algorithm, to launch the next reading ASAP
             Ask for a RAW Temp read
             When raw temp is ready, decode it and then ask for a RAW pressure read
             when RAW pressure read is ready, decode it and signal the caller
         if the caller is ready, method readAll converts RAW mesures to useful ones
         along with altitude.


 ****************************************************/

#include "Arduino.h"

#define BMP085_DEBUG 0

#define BMP085_I2CADDR 0x77

#define BMP085_ULTRALOWPOWER 0
#define BMP085_STANDARD      1
#define BMP085_HIGHRES       2
#define BMP085_ULTRAHIGHRES  3

#define BMP085_CAL_AC1           0xAA  // R   Calibration data (16 bits)
#define BMP085_CAL_AC2           0xAC  // R   Calibration data (16 bits)
#define BMP085_CAL_AC3           0xAE  // R   Calibration data (16 bits)    
#define BMP085_CAL_AC4           0xB0  // R   Calibration data (16 bits)
#define BMP085_CAL_AC5           0xB2  // R   Calibration data (16 bits)
#define BMP085_CAL_AC6           0xB4  // R   Calibration data (16 bits)
#define BMP085_CAL_B1            0xB6  // R   Calibration data (16 bits)
#define BMP085_CAL_B2            0xB8  // R   Calibration data (16 bits)
#define BMP085_CAL_MB            0xBA  // R   Calibration data (16 bits)
#define BMP085_CAL_MC            0xBC  // R   Calibration data (16 bits)
#define BMP085_CAL_MD            0xBE  // R   Calibration data (16 bits)

#define BMP085_CONTROL           0xF4 
#define BMP085_TEMPDATA          0xF6
#define BMP085_PRESSUREDATA      0xF6
#define BMP085_READTEMPCMD       0x2E
#define BMP085_READPRESSURECMD   0x34

#define BMP085_EOC_FINISHED 1
#define BMP085_EOC_RUNNING 0


typedef struct {
    float temperature; //in degrees
    int32_t pressure; //in hpa
    float altitude; // in meters
    float hpa0; //Reference pressure (ie pressure at which the alt value will be zero)
} bmpData_t;

boolean beginBMP085(uint8_t mode, uint8_t pinNumber);  // by default go highres

boolean updateBMP085Cycle(void);

void readBMP085All(float sealevelPressure,
                   bmpData_t* pt_outputData); // std atmosphere


