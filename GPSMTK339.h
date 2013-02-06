/*
 * GPSMTK339.h
 *
 *  Created on: 5 févr. 2013
 *      Author: neuf
 *
 *
 *  Main code is taken from the following book :
 *       APRESS Arduino Robotics, by John-David Warren, Josh Adams and Harald Molle
 *
 *  With modifications to match my needs :
 *   - Use with the Adafruit Ultimate GPS module using MTK33x9 chipset
 *          ------> http://www.adafruit.com/products/746
 *   - Some additional decoded data.
 *
 */

#ifndef GPSMTK339_H_
#define GPSMTK339_H_

#include "Arduino.h"


typedef struct {
    int fix; //Fix and quality, 0 no fix, 1 good fix, 2 differential fix
    int sats; //number of sats being used for the fix
    float hdop; // Horizontal dilution of precision
    float alt_m; //altitude, in meters
    uint8_t hour;
    uint8_t minute;
    uint8_t seconds;
    uint16_t milliseconds;
    float lat; //In decimal degrees
    float lon; //In decimal degrees
    float spd_kmh;  //Speed in kmh
    float heading; //heading/course/bearing, in degrees
    uint8_t day;
    uint8_t month;
    uint8_t year; //int, num of years from year 2000
} t_gpsData;

void begin_gps(void);

boolean gps_read_serial_and_parse_nmea(t_gpsData* pt_outputData);

#endif /* GPSMTK339_H_ */
