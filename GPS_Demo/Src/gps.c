/*
 * gps.c
 *
 *  Created on: Mar 15, 2019
 *      Author: Asus
 */

#include "string.h"
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "stm32l0xx_hal.h"

#include "gps.h"

static int gps_parse_nmea_sentence(GPS_t *gpsHandle, uint8_t *data, uint16_t size);
static void gps_parse_nmea_GPGGA(GPS_t *gpsHandle, uint8_t *data, uint16_t size);

GPS_t *gps_open(GPS_data_type_e dataType)
{
	GPS_t *gpsHandle = calloc(1, sizeof(GPS_t));
	if (gpsHandle != NULL)
	{
		gpsHandle->dataType = GPS_DATA_TYPE_NMEA;
	}

	return gpsHandle;
}

void gps_set_input_buffer(GPS_t *gpsHandle, uint8_t *data, uint16_t size)
{
	static uint8_t parse_result = 0;
	static uint8_t buffer[512];
	static uint8_t old_buffer[256];
	static uint16_t old_buffer_pointer = 0;
	static uint16_t new_size = 0;

	if (old_buffer_pointer == 0)
	{
		memcpy(buffer, data, size);
		new_size = size;
	}
	else
	{
		memcpy(buffer, old_buffer, old_buffer_pointer);
		memcpy(&buffer[old_buffer_pointer], data, size);
		new_size = old_buffer_pointer + size;
	}

	switch (gpsHandle->dataType)
	{
	case GPS_DATA_TYPE_NMEA:
		parse_result = gps_parse_nmea_sentence(gpsHandle, buffer, new_size);
		break;
	case GPS_DATA_TYPE_UBX:
		break;
	}

	if (!parse_result)
	{
		memcpy(&old_buffer[old_buffer_pointer], data, size);
		old_buffer_pointer += size;
	}
}

static int gps_parse_nmea_sentence(GPS_t *gpsHandle, uint8_t *data, uint16_t size)
{
	static uint16_t new_size = 0;
	static uint8_t *current_position;
	current_position = data;

	while(current_position < (data + size))
	{
		char *dollar_pointer = strstr((char *)current_position, "$");
		if (dollar_pointer == NULL)
		{
			return -1;
		}
		char *star_pointer = strstr((char *)dollar_pointer, "*");
		if (star_pointer == NULL)
		{
			return -1;
		}

		new_size = (uint8_t *)star_pointer - current_position;

		gps_parse_nmea_GPGGA(gpsHandle, current_position, new_size);

		current_position += new_size;
	}

	return 1;
}

extern UART_HandleTypeDef huart1;
static void gps_parse_nmea_GPGGA(GPS_t *gpsHandle, uint8_t *data, uint16_t size)
{
	// Parse
	char *GPGGA = strstr((char *)data, "GPGGA");
	if (GPGGA != NULL)
	{
		   const char s[2] = ",";
		   char *token;

		   uint8_t degreeLon;
		   uint8_t degreeLat;
		   float minute;
		   token = strtok(GPGGA, s);
		   uint8_t tokenIndex = 0;
		   while( token != NULL )
		   {
			   switch(tokenIndex)
			   {
			   case 0:
				   //"GPGGA"
				   break;
			   case 1:
				   //"fixTakenAt"
				   token[6] = 0;
				   gpsHandle->gpgga.fixTakenAt.Second = strtol(token+4, (char **)NULL, 10);
				   token[4] = 0;
				   gpsHandle->gpgga.fixTakenAt.Minute = strtol(token+2, (char **)NULL, 10);
				   token[2] = 0;
				   gpsHandle->gpgga.fixTakenAt.Hour = strtol(token, (char **)NULL, 10);
				   break;
			   case 2:
				   //Latitude 4807.038
				   degreeLat = strtol(token, (char **)NULL, 10)/100;
				   minute = strtof (token, NULL)-(degreeLat*100);
				   gpsHandle->gpgga.latitude = minute/60 + degreeLat;
				   break;
			   case 3:
				   //Latitude NS
				   if (token[0] == 'S')
				   {
					   gpsHandle->gpgga.latitude *= -1;
				   }
				   break;
			   case 4:
				   //Longitude 4807.038
				   degreeLon = strtol(token, (char **)NULL, 10)/100;
				   minute = strtof (token, NULL)-(degreeLon*100);
				   gpsHandle->gpgga.longitude = minute/60 + degreeLon;
				   break;
			   case 5:
				   //Longitude EW
				   if (token[0] == 'W')
				   {
					   gpsHandle->gpgga.longitude *= -1;
				   }
				   break;
			   case 6:
				   //FixQuality
				   gpsHandle->gpgga.fixQuality = strtol(token, (char **)NULL, 10);
				   break;
			   case 7:
				   //Number of satellites
				   gpsHandle->gpgga.numSatellites = strtol(token, (char **)NULL, 10);
				   break;
			   case 8:
				   //horizontalDilution
				   gpsHandle->gpgga.horizontalDilution = strtof (token, NULL);
				   break;
			   case 9:
				   //Altitude
				   gpsHandle->gpgga.altitude = strtof (token, NULL);
				   break;
			   case 10:
				   //Altitude quantity
				   break;
			   case 11:
				   // Height of geoid (mean sea level) above WGS84 ellipsoid
				   gpsHandle->gpgga.heightOfGeoid = strtof (token, NULL);
				   break;
			   case 12:
				   // Height of geoid quantity
				   break;
			   }

			   tokenIndex++;
			   //HAL_UART_Transmit(&huart1, (uint8_t *)token, strlen(token), 100);
			   token = strtok(NULL, s);
		   }

		   //gpgga.altitude = 0;
	}
}
//HAL_UART_Transmit(&huart1, (uint8_t *)data, size, 100);
//HAL_UART_Transmit(&huart1, "XX\r\n", 3, 100);

//return;














