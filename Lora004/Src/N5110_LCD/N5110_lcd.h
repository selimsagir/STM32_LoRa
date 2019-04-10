/*
 * N5110_lcd.h
 *
 *  Created on: Nov 17, 2018
 *      Author: sef
 */

#ifndef N5110_LCD_N5110_LCD_H_
#define N5110_LCD_N5110_LCD_H_

#include "fonts/fonts.h"

typedef enum
{
	N5110_OUT_OF_FRAMEBUFFER_X = -2,
	N5110_OUT_OF_FRAMEBUFFER_Y = -1,
	N5110_OK = 1,
}N5110_Status;

typedef struct
{
	uint8_t *frameBuffer;
	struct
	{
		void 		*csPort;
		uint32_t 	csPin;
		void 		*DCPort;
		uint32_t 	DCPin;
		void 		*RstPort;
		uint32_t 	RstPin;
	} pins;

	struct
	{
		void (*delay_ms)(uint32_t);
		uint32_t (*get_time_ms)(void);
		void (*set_gpio)(void*, uint32_t);
		void (*reset_gpio)(void*, uint32_t);
		void (*spi_transmit)(uint8_t*, uint32_t);
		void (*spi_transmit_dma)(uint8_t*, uint32_t);
	} functions;
} AT_N5110_LCD_handle;

AT_N5110_LCD_handle *AT_N5110LCD_open();
void AT_N5110LCD_init(AT_N5110_LCD_handle *instance);

void AT_N5110LCD_update_display(AT_N5110_LCD_handle* instance);
N5110_Status AT_N5110LCD_get_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y, uint8_t *pixel);
N5110_Status AT_N5110LCD_set_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y);
N5110_Status AT_N5110LCD_clear_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y);
N5110_Status AT_N5110LCD_draw_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y, uint8_t pixel);

N5110_Status AT_N5110LCD_print_char(AT_N5110_LCD_handle* instance, sFONT *font, uint8_t character, int32_t X, int32_t Y);
N5110_Status AT_N5110LCD_print_string(AT_N5110_LCD_handle* instance, sFONT *font, uint8_t *text, uint32_t size, int32_t X, int32_t Y);

#endif /* N5110_LCD_N5110_LCD_H_ */
