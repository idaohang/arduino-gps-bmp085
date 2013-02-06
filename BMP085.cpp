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

#include "BMP085.h"
#include "Wire.h"

/****************
 * internal types
 *****************/
typedef enum {
    NONE,
    TEMPERATURE_IN_PROGRESS,
    PRESSURE_IN_PROGRESS
} cycleStep_t;

/****************
 * Function prototypes
 *****************/

uint8_t read8(uint8_t addr);
uint16_t read16(uint8_t addr);
void write8(uint8_t addr, uint8_t data);

/***************************
 * Internal data
 *************************/

cycleStep_t currentStep = NONE;
uint8_t oversampling;
uint8_t eocPinNumber; //for End of Conversion flag readings

//the following names match the datasheet.

//BMP calibration factors
int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;
int16_t UT; //RAW temp
int32_t UP; //RAW pressure


/***************************
 * Real code starts here
 ****************************/
/**
 * Sets the sampling mode, EOC pin number, check the BMP presence then retrieve the
 * calibration factors
 *
 * return true if captor is present and init successful.
 */
boolean beginBMP085(uint8_t mode, uint8_t pin) {
    oversampling = mode;
    if (oversampling > BMP085_ULTRAHIGHRES) {
        oversampling = BMP085_ULTRAHIGHRES;
    }
    eocPinNumber = pin;

    Wire.begin();

    if (read8(0xD0) != 0x55) {
        return false;
    }

    /* read calibration data */
    ac1 = read16(BMP085_CAL_AC1);
    ac2 = read16(BMP085_CAL_AC2);
    ac3 = read16(BMP085_CAL_AC3);
    ac4 = read16(BMP085_CAL_AC4);
    ac5 = read16(BMP085_CAL_AC5);
    ac6 = read16(BMP085_CAL_AC6);

    b1 = read16(BMP085_CAL_B1);
    b2 = read16(BMP085_CAL_B2);

    mb = read16(BMP085_CAL_MB);
    mc = read16(BMP085_CAL_MC);
    md = read16(BMP085_CAL_MD);

    return true;
}

/********************
 * Main sequencing algorithm, designed to be included in the main loop.
 * Implements the following sequence
 *    Ask for a RAW Temp read
 *    When raw temp is ready, decode it and then ask for a RAW pressure read
 *    When RAW pressure read is ready, decode it and signal the caller by a true return
 *
 * State change is done by polling at each call the EOC pin value.
 *
 *******************/
boolean updateBMP085Cycle(void) {
    boolean cycleComplete = false;

    switch (currentStep) {
        case NONE :
            //start with a temp read
            write8(BMP085_CONTROL, BMP085_READTEMPCMD);
            currentStep = TEMPERATURE_IN_PROGRESS;
            break;
        case TEMPERATURE_IN_PROGRESS :
            if (digitalRead(eocPinNumber) > 0) {
                //temp read complete, updating data !
                UT = read16(BMP085_TEMPDATA);

                //reading pressure next
                write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));
                currentStep = PRESSURE_IN_PROGRESS;
            }
            //else conversion is still running, about 5ms in high res
            break;
        case PRESSURE_IN_PROGRESS :
            if (digitalRead(eocPinNumber) > 0) {
                //temp read complete, updating data !
                UP = read16(BMP085_PRESSUREDATA);

                UP <<= 8;
                UP |= read8(BMP085_PRESSUREDATA + 2);
                UP >>= (8 - oversampling);

                //reading temperature next
                write8(BMP085_CONTROL, BMP085_READTEMPCMD);
                currentStep = TEMPERATURE_IN_PROGRESS;

                //And a cycle has been completed
                cycleComplete = true;
            }
            //else conversion is still running, about 5ms (low pow) to 26ms (high res)
            break;
        default :
            //should not happen
            break;
    }
    return cycleComplete;
}


/**
 * Convert current raw values to the understable values.
 * Values might be outdated if call is done in between
 * true return of updateBMP085Cycle() calls.
 */
void readBMP085All(float sealevelPressure,
                        bmpData_t* pt_outputData) {
    int32_t B3, B5, B6, X1, X2, X3;
    uint32_t B4, B7;

    // do temperature calculations
    X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) / pow(2,15);
    X2 = ((int32_t)mc * pow(2,11)) / (X1+(int32_t)md);
    B5 = X1 + X2;

    pt_outputData->temperature = (B5+8)/pow(2,4);
    pt_outputData->temperature /= 10;

    // do pressure calcs
    B6 = B5 - 4000;
    X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
    X2 = ((int32_t)ac2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

    X1 = ((int32_t)ac3 * B6) >> 13;
    X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

    if (B7 < 0x80000000) {
        pt_outputData->pressure = (B7 * 2) / B4;
    } else {
        pt_outputData->pressure = (B7 / B4) * 2;
    }

    X1 = (pt_outputData->pressure >> 8) * (pt_outputData->pressure >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * pt_outputData->pressure) >> 16;

    pt_outputData->pressure = pt_outputData->pressure + ((X1 + X2 + (int32_t)3791)>>4);

    pt_outputData->altitude = 44330 * (1.0 - pow(((float)pt_outputData->pressure) / sealevelPressure, 0.1903));

}

/**************************
 * Utility methods.
 **************************/

uint8_t read8(uint8_t a) {
    uint8_t ret;

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.endTransmission(); // end transmission

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
    ret = Wire.read(); // receive DATA
    Wire.endTransmission(); // end transmission

    return ret;
}

uint16_t read16(uint8_t a) {
    uint16_t ret;

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.endTransmission(); // end transmission

    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
    ret = Wire.read(); // receive DATA
    ret <<= 8;
    ret |= Wire.read(); // receive DATA
    Wire.endTransmission(); // end transmission

    return ret;
}

void write8(uint8_t a, uint8_t d) {
    Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
    Wire.write(a); // sends register address to read from
    Wire.write(d);  // write data
    Wire.endTransmission(); // end transmission
}
