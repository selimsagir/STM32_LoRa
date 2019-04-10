/*
 * gps.h
 *
 *  Created on: Mar 15, 2019
 *      Author: Asus
 */

#ifndef GPS_H_
#define GPS_H_

typedef enum
{
	GPS_DATA_TYPE_NMEA,
	GPS_DATA_TYPE_UBX
} GPS_data_type_e;


typedef struct
{
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint16_t Millisecond;

	uint16_t Year;
	uint8_t Month;
	uint8_t Day;
} gps_time_t;

typedef enum
{
	GPS_FIX_QUALITY_INVALID,
	GPS_FIX_QUALITY_GPS_FIX,
	GPS_FIX_QUALITY_DGPS_FIX,
	GPS_FIX_QUALITY_PPS_FIX,
	GPS_FIX_QUALITY_RTK,
	GPS_FIX_QUALITY_FRTK,
	GPS_FIX_QUALITY_ESTIMATED,
	GPS_FIX_QUALITY_MANUAL_INPUT,
	GPS_FIX_QUALITY_SIM_MODE
} GPS_fix_quality;

typedef struct
{
	gps_time_t fixTakenAt;
	float latitude;
	float longitude;
	GPS_fix_quality fixQuality;
	uint8_t numSatellites;
	float	horizontalDilution;
	float	altitude;
	float	heightOfGeoid;
} GPS_GPGGA;

typedef struct
{
	GPS_data_type_e dataType;
	GPS_GPGGA gpgga;
} GPS_t;

GPS_t *gps_open(GPS_data_type_e dataType);

void gps_set_input_buffer(GPS_t *gpsHandle, uint8_t *data, uint16_t size);

#endif /* GPS_H_ */
