/*
 * font.h
 *
 *  Created on: Nov 17, 2018
 *      Author: sef
 */

#ifndef N5110_LCD_FONTS_H_
#define N5110_LCD_FONTS_H_

#include <stdint.h>

typedef struct _tFont
{
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;

} sFONT;
extern sFONT FontMenlo32;
extern sFONT Font24;
extern sFONT Font20;
extern sFONT Font16;
extern sFONT Font12;
extern sFONT Font8;

#endif /* N5110_LCD_FONTS_H_ */
