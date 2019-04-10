/*
 * N5110_lcd.c
 *
 *  Created on: Nov 17, 2018
 *      Author: Sefa Unal
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "N5110_lcd.h"
#include "N5110_lcd_defs.h"



static void AT_N5110LCD_init_hw();
static void AT_N5110LCD_write_buffer(AT_N5110_LCD_handle* instance, uint8_t *data, uint32_t size);
static void AT_N5110LCD_write_buffer_dma(AT_N5110_LCD_handle* instance, uint8_t *data, uint32_t size);
static void AT_N5110LCD_set_reset(AT_N5110_LCD_handle* instance);
static void AT_N5110LCD_clear_reset(AT_N5110_LCD_handle* instance);
static void AT_N5110LCD_reset_duration(AT_N5110_LCD_handle* instance, uint32_t duration_ms);
static void AT_N5110LCD_send_data(AT_N5110_LCD_handle* instance, uint8_t data);
static void AT_N5110LCD_send_command(AT_N5110_LCD_handle* instance, uint8_t data);
static void AT_N5110LCD_clear(AT_N5110_LCD_handle* instance);

AT_N5110_LCD_handle *AT_N5110LCD_open()
{
	AT_N5110_LCD_handle *instance = (AT_N5110_LCD_handle*)malloc(sizeof (AT_N5110_LCD_handle));

	if (instance != NULL)
	{
	}

	return instance;
}

void AT_N5110LCD_init(AT_N5110_LCD_handle *instance)
{
	if (instance != NULL)
	{
		instance->frameBuffer = (uint8_t*)malloc(84*84);  //0x30000000
		memset(instance->frameBuffer, 0, N5110_TOTAL_ADDR_SIZE);

		AT_N5110LCD_init_hw(instance);
	}
}
static void AT_N5110LCD_init_hw(AT_N5110_LCD_handle* instance)
{
	AT_N5110LCD_reset_duration(instance, 500);
	AT_N5110LCD_send_command(instance, N5110_FS | N5110_FS_EXTENDED_INST_SET);
	AT_N5110LCD_send_command(instance, N5110_BS | 3);
	AT_N5110LCD_send_command(instance, N5110_SETVOP | 0x3F);
	AT_N5110LCD_send_command(instance, N5110_FS);
	AT_N5110LCD_send_command(instance, N5110_DC | N5110_DC_NORMAL_MODE);
	AT_N5110LCD_clear(instance);
}

void AT_N5110LCD_update_display(AT_N5110_LCD_handle* instance)
{
	AT_N5110LCD_send_command(instance, N5110_SETY | 0);
	AT_N5110LCD_send_command(instance, N5110_SETX | 0);

	instance->functions.set_gpio(instance->pins.DCPort, instance->pins.DCPin);
	AT_N5110LCD_write_buffer_dma(instance, instance->frameBuffer, N5110_TOTAL_ADDR_SIZE);
}

N5110_Status AT_N5110LCD_get_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y, uint8_t *pixel)
{
	if (X > N5110_WIDTH_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_X;
	if (Y > N5110_HEIGHT_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_Y;

	*pixel = (instance->frameBuffer[X + (Y/8) * N5110_X_ADDR_LENGTH] & (1 << (Y%8)));
	return N5110_OK;
}

N5110_Status AT_N5110LCD_set_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y)
{
	if (X > N5110_WIDTH_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_X;
	if (Y > N5110_HEIGHT_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_Y;

	instance->frameBuffer[X + (Y/8) * N5110_X_ADDR_LENGTH] |= 1 << (Y%8);
	return N5110_OK;
}

N5110_Status AT_N5110LCD_clear_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y)
{
	if (X > N5110_WIDTH_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_X;
	if (Y > N5110_HEIGHT_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_Y;

	instance->frameBuffer[X + (Y/8) * N5110_X_ADDR_LENGTH] &= ~(1 << (Y%8));
	return N5110_OK;
}

N5110_Status AT_N5110LCD_draw_pixel(AT_N5110_LCD_handle* instance, uint8_t X, uint8_t Y, uint8_t pixel)
{
	if (pixel > 0)
	{
		return AT_N5110LCD_set_pixel(instance, X, Y);
	}
	else
	{
		return AT_N5110LCD_clear_pixel(instance, X, Y);
	}
}

#define INTEGER_DIV_CEIL(x, y)  (x/y + (x % y != 0))

N5110_Status AT_N5110LCD_print_char(AT_N5110_LCD_handle* instance, sFONT *font, uint8_t character, int32_t X, int32_t Y)
{
	if (X > N5110_WIDTH_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_X;
	if (Y > N5110_HEIGHT_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_Y;

	for (int k=0; k<font->Height; k++)
	{
		for (int s=0; s<INTEGER_DIV_CEIL(font->Width, 8); s++)
		{
			uint8_t sutunx = font->table[INTEGER_DIV_CEIL(font->Width, 8) * font->Height * (character - 0x20) + k*INTEGER_DIV_CEIL(font->Width, 8) + s];
			for (int l=0; l<8; l++)
			{
				if (AT_N5110LCD_draw_pixel(instance, X + s*8 + l, Y + k, sutunx & (0x80>>l)) == N5110_OUT_OF_FRAMEBUFFER_Y)
				{
					return N5110_OK;
				}
			}
		}
	}
	return N5110_OK;
}

N5110_Status AT_N5110LCD_print_string(AT_N5110_LCD_handle* instance, sFONT *font, uint8_t *text, uint32_t size, int32_t X, int32_t Y)
{
	if (X > N5110_WIDTH_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_X;
	if (Y > N5110_HEIGHT_PIXELS)
		return N5110_OUT_OF_FRAMEBUFFER_Y;

	for(int i=0; i<size; i++)
	{
		N5110_Status result = AT_N5110LCD_print_char(instance, font, text[i], X + font->Width*i, Y);
		if (result != N5110_OK)
			return result;
	}

	return N5110_OK;
}

static void AT_N5110LCD_write_buffer(AT_N5110_LCD_handle* instance, uint8_t *data, uint32_t size)
{
	instance->functions.reset_gpio(instance->pins.csPort, instance->pins.csPin);
	instance->functions.spi_transmit(data, size);
	instance->functions.set_gpio(instance->pins.csPort, instance->pins.csPin);
}

static void AT_N5110LCD_write_buffer_dma(AT_N5110_LCD_handle* instance, uint8_t *data, uint32_t size)
{
	instance->functions.spi_transmit_dma(data, size);
}

static void AT_N5110LCD_set_reset(AT_N5110_LCD_handle* instance)
{
	instance->functions.reset_gpio(instance->pins.RstPort, instance->pins.RstPin);
}

static void AT_N5110LCD_clear_reset(AT_N5110_LCD_handle* instance)
{
	instance->functions.set_gpio(instance->pins.RstPort, instance->pins.RstPin);
}

static void AT_N5110LCD_reset_duration(AT_N5110_LCD_handle* instance, uint32_t duration_ms)
{
	static uint32_t timeBefore;
	timeBefore = instance->functions.get_time_ms();
	AT_N5110LCD_set_reset(instance);

	// wait for a while
	while (instance->functions.get_time_ms() - timeBefore < duration_ms );

	AT_N5110LCD_clear_reset(instance);
}

static void AT_N5110LCD_send_data(AT_N5110_LCD_handle* instance, uint8_t data)
{
	instance->functions.set_gpio(instance->pins.DCPort, instance->pins.DCPin);
	AT_N5110LCD_write_buffer(instance, &data, 1);
}

static void AT_N5110LCD_send_command(AT_N5110_LCD_handle* instance, uint8_t data)
{
	instance->functions.reset_gpio(instance->pins.DCPort, instance->pins.DCPin);
	AT_N5110LCD_write_buffer(instance, &data, 1);
}

static void AT_N5110LCD_clear(AT_N5110_LCD_handle* instance)
{
	for(int i=0; i<N5110_TOTAL_ADDR_SIZE; i++)
		AT_N5110LCD_send_data(instance, 0x00);
}
