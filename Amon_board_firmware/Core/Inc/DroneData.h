/*
 * DroneData.h
 *
 *  Created on: May 17, 2023
 *      Author: titit
 */

#ifndef INC_DRONEDATA_H_
#define INC_DRONEDATA_H_


// Connect values with status codes for RF
#define STATUS_STARTUP		0	/* status when power on */
#define STATUS_IDLE_NC		1 	/* not connected */
#define STATUS_IDLE_CN		2 	/* connected */
#define STATUS_ERROR		3	/* Error */
#define STATUS_ARM			4	/* State before flight */
#define STATUS_FLY			5	/* Flying */
#define STATUS_FLY_OVER		6	/* After slight */
#define STATUS_GYRO_CALIB	7	/* When calibrating gyro, not for flight */

#define RF_STATUS_CN		1	/* RF connected */
#define RF_STATUS_NC		0	/* RF not connected */

#define CALIBRATION			0	/* Change to 1 to enable gyro calibration mode */
#define CAL_GYRO_X			0
#define CAL_GYRO_Y			0
#define CAL_GYRO_Z			0
#define CAL_ACCEL_X			0
#define CAL_ACCEL_Y			0
#define CAL_ACCEL_Z			0
#define CAL_PITCH			0
#define CAL_ROLL			0
#define CAL_LIDAR			0

#define RADIO_NUM			2 	/* Set number of radios mounted on board */

// same defines as in PWM.h
#define SERVO_XP 			1
#define SERVO_XN 			2
#define SERVO_YP 			3
#define SERVO_YN 			4
#define PWM_EDF 			5

/*
 * Select GPS decoding (1-enable, 0-disable)
 */
#define USE_GPS				1
#define USE_GPS_GGA 		1
#define USE_GPS_GLL 		0
#define USE_GPS_GSA 		0
#define USE_GPS_GSV 		0
#define USE_GPS_RMC 		0
#define USE_GPS_VTG 		0




typedef struct {

	/* Basic Data */
	uint8_t DroneStatus;	 	/* Status of drone */

	uint16_t MainBatVoltage;	/* Voltage of main board battery */

	uint16_t EDFBatVoltage;		/* Voltage of EDF fan battery */

	uint8_t Connected;			/* Status of connection with ground station */

	uint16_t Time;				/* Current time - GPS */

	uint16_t Date;				/* Current date - GPS */

	/* Orientation of drone */
	float Pitch;
	float PitchOld;

	float Roll;
	float RollOld;

	float Yaw;
	float YawOld;

	/* Data before flight to initialize orientation */
	float PitchMean;
	float RollMean;

	/* Height of drone (when on ground the height is 0, offset on sensor set to 130mm) */
	uint16_t Height;

	/* Data from GPS*/


} AMON_Drone;



#endif /* INC_DRONEDATA_H_ */
