/**
* GPS SD logger.
* Connexions :
*     - Connect the GPS Power pin to 5V
*     - Connect the GPS Ground pin to ground
*     - Connect the GPS TX (transmit) pin to Digital 3
*     - Connect the GPS RX (receive) pin to Digital 2
*
* SD :
*     - MOSI : 12
*     - MISO : 11
*     - SCK  : 13
*     - CS   : 10
*
* BMP
*     - SCL
*     - SDA
*     - EOC : to determine.
*/


/***************************************************
* INCLUDES
***************************************************/


#include "Arduino.h"
#include "SD.h"
#include "BMP085.h"
#include "GPSMTK339.h"

/***************************************************
* DEFINES
***************************************************/
/*
* Serial (debug only)
*/
#define SERIAL_ACTIVE true
#define SERIAL_SPEED 115200

/*
* Log SD
*/
#define SEPARATOR '|'
#define FOLDER (char*)"LOGS_GPS"
#define MYFILE (char*)"LOGS_GPS/HZ1_02.csv"

/*
 * BMP085
 */
#define SEA_LEVEL_PRESSURE ((float)101325.0)

/*
* Pins
*/
#define PIN_LED_GREEN 6
#define PIN_LED_RED 7
#define PIN_SWITCH_CALIB 4
#define PIN_EOC 8


/***************************************************
* DATA
***************************************************/

//SD
int const chipSelect = 10;

bmpData_t bmp085Data;

t_gpsData gps_data;

/***************************************************
* Functions declaration
***************************************************/
void fatal_error(void);
void fatal_error_overflow(void);
void writeGpsData(void);


/***************************************************
* Functions
***************************************************/

/*
 * Arduino init function
 */
void setup()
{
    //Pin setting
    pinMode(PIN_SWITCH_CALIB, INPUT);
    pinMode(PIN_EOC, INPUT);

    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(SS, OUTPUT);

    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);

    bmp085Data.hpa0 = SEA_LEVEL_PRESSURE;

    //BMP Init
    if (!beginBMP085(BMP085_HIGHRES, PIN_EOC)) {
        fatal_error();
    }

    //SD card init
    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect)) {
        fatal_error();
    }

    // Open up the file we're going to log to!
    File dataFile = SD.open(MYFILE, FILE_WRITE);
    if (!dataFile) {
        fatal_error_overflow();
    }
    dataFile.println("Fix|sats|HDOP|alt(m)|Date|Time|lat|Long|Spd(kmh)|Head|temp|hpa|hpa0|alt|");
    dataFile.flush();
    dataFile.close();
    delay(1000);
    digitalWrite(PIN_LED_GREEN, LOW);
}

/*************************************************************************
* Main arduino loop
*************************************************************************/
void loop() {

    if (updateBMP085Cycle()) {
        readBMP085All(bmp085Data.hpa0, &bmp085Data);
    }

    if (gps_read_serial_and_parse_nmea(&gps_data)) {
        writeGpsData();
    }
}


/*************************************************************************
* RED LED on, GREEN LED Off, while true
* This function is a dead end
*************************************************************************/
void fatal_error(void) {
    // don't do anything more:
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, LOW);
    while (1) ;
}

/*************************************************************************
* RED LED on, GREEN LED on, while true
* This function is a dead end
*************************************************************************/
void fatal_error_overflow(void) {
    // don't do anything more:
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_LED_GREEN, HIGH);
    while (1) ;
}

/*************************************************************************
 * Dumps BMP data and GPS data to the SD card.
*************************************************************************/
void writeGpsData(void) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    File dataFile = SD.open(MYFILE, FILE_WRITE);

    //GGA
    dataFile.print(gps_data.fix);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.sats);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.hdop);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.alt_m);
    dataFile.print(SEPARATOR);
    //RMC
    dataFile.print("20");
    dataFile.print(gps_data.year);
    dataFile.print(gps_data.month);
    dataFile.print(gps_data.day);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.hour);
    dataFile.print(gps_data.minute);
    dataFile.print(gps_data.seconds);
    dataFile.print(".");
    dataFile.print(gps_data.milliseconds);
    dataFile.print(SEPARATOR);

    dataFile.print(gps_data.lat,8);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.lon,8);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.spd_kmh);
    dataFile.print(SEPARATOR);
    dataFile.print(gps_data.heading);
    dataFile.print(SEPARATOR);
    //BMP
    dataFile.print(bmp085Data.temperature);
    dataFile.print(SEPARATOR);
    dataFile.print(bmp085Data.pressure);
    dataFile.print(SEPARATOR);
    dataFile.print(bmp085Data.hpa0);
    dataFile.print(SEPARATOR);
    dataFile.print(bmp085Data.altitude);
    dataFile.print(SEPARATOR);

    dataFile.println();
    dataFile.flush();
    dataFile.close();
    digitalWrite(PIN_LED_GREEN, LOW);
}
