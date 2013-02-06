/*
 * GPSMTK339.cpp
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

#include "GPSMTK339.h"
#include "avr/pgmspace.h"

/*
* GPS Commands
*/
// different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ  (char*)("$PMTK220,1000*1F")
#define PMTK_SET_NMEA_UPDATE_5HZ  (char*)("$PMTK220,200*2C")
#define PMTK_SET_NMEA_UPDATE_10HZ (char*)("$PMTK220,100*2F")

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600  "$PMTK251,9600*17"


#define PMTK_SET_NMEA_OUTPUT_RMCONLY (char*)("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29")// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCGGA  (char*)("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28") // turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA (char*)("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28")// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_OFF     (char*)("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28")// turn off output

#define GPS_BUFFER_SIZE 120 //In number of ASCII char

/***************************************************
* DATA
***************************************************/
const char *search = (char*)(",");

char buffer[GPS_BUFFER_SIZE]; //Serial buffer to catch GPS data


/***************************************************
* FUNCTIONS PROTOTYPES
***************************************************/
boolean parse_rmc(t_gpsData* pt_outputData);
boolean parse_gga(t_gpsData* pt_outputData);


/***************************************************
* FUNCTIONS
***************************************************/

/***************
 * Init the chip.
 */
void begin_gps(void) {
    //TODO add some more user params...
    //GPS init
    Serial.begin(9600);
    delay(10);
    Serial.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    Serial.println(PMTK_SET_NMEA_UPDATE_1HZ);
}

/*************************************************************************
* This functions parses the NMEA strings...
* Pretty complex but never fails and works well with all GPS modules and baud speeds.. :-)
*
* return true if A valid NMEA sentence was decoded and the t_gpsData given was
* updated. Update can be partial depending on the sentence.
*************************************************************************/
boolean gps_read_serial_and_parse_nmea(t_gpsData* pt_gps_data)
{
    static byte unlock=1; //some kind of event flag
    static byte counter=0; //general counter
    static boolean rmc_ready = false;
    static boolean gga_ready = false;

    boolean res = false;

    while(Serial.available() > 0)
    {
        if(unlock==0)
        {
            buffer[0]=Serial.read();//puts a byte in the buffer
            if(buffer[0]=='$')//Verify if is the preamble $
            {
                unlock=1;
            }
        }
        else
        {
            buffer[counter]=Serial.read();
            if(buffer[counter]==0x0A) //Looks for \F
            {
                unlock=0;


                rmc_ready = rmc_ready | parse_rmc(pt_gps_data);

                gga_ready = gga_ready | parse_gga(pt_gps_data);

                if (gga_ready && rmc_ready) {
                    gga_ready = false;
                    rmc_ready = false;
                    res = true;
                }

                for(int a=0; a<=counter; a++)//restarting the buffer
                {
                    buffer[a]=0;
                }
                counter=0; //Restarting the counter
            }
            else
            {
                counter++; //Incrementing counter
// TODO check for overflow
//                if (counter > GPS_BUFFER_SIZE) {
//                    fatal_error_overflow();
//                }
            }
        }
    }
    return res;
}


/*************************************************************************
* This functions parses the GCA string.
* Table-2: GGA Data Format
 * Name                    Example      Units/Description
 * Message ID              $GPGGA       GGA protocol header
 * UTC Time                064951.000   hhmmss.sss
 * Latitude                2307.1256    ddmm.mmmm
 * N/S Indicator           N            N=north or S=south
 * Longitude               12016.4438   dddmm.mmmm
 * E/W Indicator           E            E=east or W=west
 * Position  Fix Indicator 1            0=Fix not available, 1=GPS fix , 2=Differential GPS fix
 * Satellites Used         8            Range 0 to 14
 * HDOP                    0.95         Horizontal Dilution of Precision
 * MSL Altitude            39.9         meters  Antenna Altitude above/below mean-sea-level
 * Units                   M            meters  Units of antenna altitude
 * Geoidal Separation      17.8         meters
 * Units                   M            meters  Units of geoids separation
 * Age of Diff. Corr.                   second  Null fields when DGPS is not used
 * Checksum                *65
 * <CR> <LF>      End of message termination
 *
 * return true if GPGGA NMEA sentence was successfully decoded, false otherwise.
 *  if true, only the following fields are updated :
 *   fix, sats, hdop, atl_m
 *************************************************************************/
boolean parse_gga(t_gpsData* pt_outputData) {
    const  char  head_gga[]        = "GPGGA"; //GPS NMEA header to look for
    static byte  checksum          = 0; //the checksum generated
    static byte  checksum_received = 0; //Checksum received
    char        *token, *brkb;
    boolean res = false;
    if (strncmp(buffer, head_gga, 5) == 0)
    {
        /*Generating and parsing received checksum, */
        for(int x=0; x<100; x++)
        {
            if(buffer[x] == '*')
            {
                checksum_received = strtol(&buffer[x+1], NULL, 16);//Parsing received checksum...
                break;
            }
            else
            {
                checksum ^= buffer[x]; //XOR the received data...
            }
        }

        if(checksum_received == checksum)//Checking checksum
        {
            token = strtok_r(buffer, search, &brkb); //GPGGA header, not used anymore
            token = strtok_r(NULL,   search, &brkb); //UTC, not used!!
            token = strtok_r(NULL,   search, &brkb); //lat, not used!!
            token = strtok_r(NULL,   search, &brkb); //north/south, nope...
            token = strtok_r(NULL,   search, &brkb); //lon, not used!!
            token = strtok_r(NULL,   search, &brkb); //wets/east, nope
            token = strtok_r(NULL,   search, &brkb); //Position fix, used!!
            pt_outputData->fix = atoi(token);

            token = strtok_r(NULL, search, &brkb); //sats in use, used!!
            pt_outputData->sats = atoi(token);

            token = strtok_r(NULL, search, &brkb);//HDOP, not needed
            pt_outputData->hdop = atof(token);

            token = strtok_r(NULL, search, &brkb);//ALTITUDE, is the only meaning of this string.. in meters of course.
            pt_outputData->alt_m = atof(token);

            res = true;
        }
        checksum=0; //Restarting the checksum
    }
    return res;
}



/*************************************************************************
* This functions parses the RMC string, and dumps relevant data to the file.
Name                Example      Units       Description
Message ID          $GPRMC      RMC protocol header
UTC Time            064951.000  hhmmss.sss
Status              A           A =data valid or V=data not valid
Latitude            2307.1256   ddmm.mmmm
N/S Indicator       N           N=north or S=south
Longitude           12016.4438  dddmm.mmmm
E/W Indicator       E           E=east or W=west
Speed over ground   0.03        knots
Course over Ground  165.48      degrees  True
Date                260406      ddmmyy
Magnetic Variation  3.05,       E=east or W=west
Mode                A           A= Autonomous mode D= Differential mode E= Estimated mode
Checksum            *2C
<CR> <LF>           End of message termination

 * return true if GPRMC NMEA sentence was successfully decoded, false otherwise.
 *  if true, only the following fields are updated :
 *   hour, minute, seconds, millis, lat, lon, speed, heading, day, month, year
*************************************************************************/
boolean parse_rmc(t_gpsData* pt_outputData) {
    const  char  head_rmc[]        = "GPRMC"; //GPS NMEA header to look for
    static byte  checksum          = 0;
    static byte  checksum_received = 0;

    char        *token, *brkb, *pEnd;
    boolean res = false;
    unsigned long temp          = 0;
    unsigned long temp2         = 0;
    unsigned long temp3         = 0;
    float         latitude_dec  = 0;
    float         longitude_dec = 0;


    if (strncmp (buffer, head_rmc, 5) == 0)
    {
        // $GPRMC parsing starts here
        /*Generating and parsing received checksum, */
        for(int x=0; x<100; x++)
        {
            if(buffer[x]=='*')
            {
                checksum_received=strtol(&buffer[x+1],NULL,16);//Parsing received checksum...
                break;
            }
            else
            {
                checksum ^= buffer[x]; //XOR the received data...
            }
        }
        if(checksum_received == checksum)//Checking checksum
        {

            /*
             * Token will point to the data between comma "'", returns the data in the order received
             * THE GPRMC order is: UTC, UTC status ,Lat, N/S indicator, Lon, E/W indicator, speed, course, date, mode, checksum
             */
            token = strtok_r(buffer, search, &brkb); //Contains the header GPRMC, not used
            token = strtok_r(NULL,   search, &brkb); //UTC Time
            float timef = atof(token);
            uint32_t time = timef;
            pt_outputData->hour = time / 10000;
            pt_outputData->minute = (time % 10000) / 100;
            pt_outputData->seconds = (time % 100);
            pt_outputData->milliseconds = fmod(timef, 1.0) * 1000;

            token = strtok_r(NULL, search, &brkb); //Status ? not used...


            /*
             * Lat/Long in degrees, decimal minutes
             * To convert to decimal degrees, divide the minutes by 60 (including decimals)
             */
            token = strtok_r(NULL, search, &brkb);
            //taking only degrees, and minutes without decimals,
            temp = strtol (token, &pEnd, 10);
            //takes only the decimals of the minutes
            temp2 = strtol (pEnd + 1, NULL, 10);
            //joining degrees, minutes, and the decimals of minute, now without the point...
            temp3 = (temp * 10000) + (temp2);
            //modulo to leave only the decimal minutes, eliminating only the degrees..
            temp3 = temp3 % 1000000;
            //Dividing to obtain only the degrees, before was 4750
            temp /= 100;
            //Joining everything and converting to float variable...
            //First i convert the decimal minutes to degrees decimals stored in "temp3", example: 501234/600000= .835390
            //Then i add the degrees stored in "temp" and add the result from the first step, example 47+.835390=47.835390
            //The result is stored in "lat" variable...
            latitude_dec = temp + ( (float)temp3 / 600000 );
            token = strtok_r(NULL, search, &brkb); //lat, north or south?
            //If the char is equal to S (south), multiply the result by -1..
            if(*token == 'S')
            {
                latitude_dec = latitude_dec * -1;
            }
            pt_outputData->lat = latitude_dec;



            //This the same procedure use in lat, but now for Lon....
            token = strtok_r(NULL, search, &brkb);
            temp = strtol (token, &pEnd, 10);
            temp2 = strtol (pEnd + 1, NULL, 10);
            temp3 = (temp * 10000) + (temp2);
            temp3 = temp3 % 1000000;
            temp /= 100;
            longitude_dec=temp + ((float)temp3 / 600000);
            token = strtok_r(NULL, search, &brkb); //lon, east or west?
            if(*token == 'W')
            {
                longitude_dec=longitude_dec * -1;
            }
            pt_outputData->lon = longitude_dec;

            token = strtok_r(NULL, search, &brkb); //Speed overground?
            pt_outputData->spd_kmh = atof(token) * 1.852;

            token = strtok_r(NULL, search, &brkb); //Course?
            pt_outputData->heading = atof(token);

            token = strtok_r(NULL, search, &brkb); //Date?
            uint32_t fulldate = atof(token);
            pt_outputData->day = fulldate / 10000;
            pt_outputData->month = (fulldate % 10000) / 100;
            pt_outputData->year = (fulldate % 100);
            res = true;
        }
        checksum=0;
    } //End of the GPRMC parsing
    return res;
}
