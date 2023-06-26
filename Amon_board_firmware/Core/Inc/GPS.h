/*
 * GPS.h
 *
 *  Created on: Apr 10, 2023
 *      Author: titit
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f4xx_hal.h"
#include "DroneData.h"


/*
 * GPS packed - GGA format
 */
typedef struct {

	uint8_t MessageID[6];		// $GPGGA
	uint8_t UTCTime[10];		// hhmmss.sss
	float Latitude;				// ddmm.mmmm
	char NSIncicator;			// N = nort / S = south
	float Longitude;			// dddmm.mmmm
	char EWIndicator;			// E = east / W = west
	uint8_t PositionFix;		// 0-6 values (3-5 not supported)
	uint8_t NoSatelites;		// 0-12
	float HDOP;					// Horizontal Dilution of Precision
	float MSLAltitude;			// altitude
	char unitsAlt;				// M - meters
	float Geoid;				// Geoid-to-ellipsoid separation. Ellipsoid altitude = MSL Altitude + Geoid Separation
	char unitsGeoid;			// M - meters
	uint8_t	AgeOfDiffCorr;		// can be null(when DGPS not used) (unit: sec)
	uint8_t	DiffRefStationId;	// 0000
	uint8_t checksum;			// chacksum

} GPS_GGA;


/*
 * GPS packed - GLL format
 */
typedef struct {

	uint8_t MessageID[6];		// $GPGGA
	float Latitude;				// ddmm.mmmm
	char NSIncicator;			// N = nort / S = south
	float Longitude[10];		// dddmm.mmmm
	char EWIndicator;			// E = east / W = west
	uint8_t UTCTime[10];		// hhmmss.sss
	char Status;				// A=data valid / V=data not valid
	char Mode;					// A = Autonomous, D = DGPS, E = DR (Only present in NMEA v3.00)
	uint8_t checksum;			// chacksum

} GPS_GLL;


/*
 * GPS packed - GSA format
 */
typedef struct {

	uint8_t MessageID[6]; 		// "$GPGSA"
	char Mode1;					// A - 2D Automatic / M - manual
	uint8_t Mode2;				// 1 - Fix not available / 2 - 2D (<4 SVs used) / 3 - 3D (>3 SVs used)
	uint8_t SatUsed1;			// SV on Channel 1
	uint8_t SatUsed2;			// SV on Channel 2
	uint8_t SatUsed3;			// SV on Channel 3
	uint8_t SatUsed4;			// SV on Channel 4
	uint8_t SatUsed5;			// SV on Channel 5
	uint8_t SatUsed6;			// SV on Channel 6
	uint8_t SatUsed7;			// SV on Channel 7
	uint8_t SatUsed8;			// SV on Channel 8
	uint8_t SatUsed9;			// SV on Channel 9
	uint8_t SatUsed10;			// SV on Channel 10
	uint8_t SatUsed11;			// SV on Channel 11
	uint8_t SatUsed12;			// SV on Channel 12
	float PDOP;					// Position Dilution of Precision
	float HDOP;					// Horizontal Dilution of Precision
	float VDOP;					// Vertical Dilution of Precision
	uint8_t checksum;			// chacksum

} GPS_GSA;


/*
 * GPS packed - GSV format
 */
typedef struct {
	uint8_t MessageID[6]; 		// "$GPGSV"
	uint8_t NumOfMess;			// range 1 to 3
	uint8_t MessNum;			// range 1 to 3
	uint8_t NoSat;				// number of satelites in view
	uint8_t SatID1;
	uint8_t Elevation1;			// degress
	uint8_t Azimuth1;			// degress
	uint8_t SNR1;				// Range 0 to 99, null when not tracking
	// Multiply last 4 variables by number of satelites in view

	uint8_t checksum;			// chacksum

} GPS_GSV;


/*
 * GPS packed - RMC format
 */
typedef struct {
	uint8_t MessageID[6]; 		// "$GPRMC"
	uint8_t UTCTime[10];		// hhmmss.sss
	char Status;				// A=data valid / V=data not valid
	float Latitude;				// ddmm.mmmm
	char NSIncicator;			// N = nort / S = south
	float Longitude[10];		// dddmm.mmmm
	char EWIndicator;			// E = east / W = west
	float Speed;				// knots
	float Course;				// degress
	uint8_t Date[6];			// ddmmyy
	uint8_t MagVar;				// E=east or W=west
	char MagEWIndicator;		// E=east
	char Mode;					// A = Autonomous, D = DGPS, E = DR
	uint8_t checksum;			// chacksum

} GPS_RMC;


/*
 * GPS packed - VTG format
 */
typedef struct {
	uint8_t MessageID[6]; 		// "$GPVTG"
	float Course1;				// degress
	uint8_t Ref1;				// True
	float Course2;				// degress
	uint8_t Ref2;				// Magnetic
	float SpeedKn;				// knots
	char UnitsKn;				// N = knots
	float SpeedKmH;				// knots
	char UnitsKmH;				// K = km / h
	char Mode;					// A = Autonomous, D = DGPS, E = DR
	uint8_t checksum;			// chacksum

} GPS_VTG;


/* FUNCTIONS */
void GPS_Decode(uint8_t *GPSData[426], GPS_GGA *gps_gga, GPS_GLL *gps_gll, GPS_GSA *gps_gsa, GPS_GSV *gps_gsv, GPS_RMC *gps_rmc, GPS_VTG *gps_vtg, AMON_Drone *AmonDrone);

#endif /* INC_GPS_H_ */
