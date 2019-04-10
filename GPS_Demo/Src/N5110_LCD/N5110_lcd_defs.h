/*
 * N5110_lcd_defs.h
 *
 *  Created on: Nov 17, 2018
 *      Author: Sefa Unal
 */

#ifndef N5110_LCD_N5110_LCD_DEFS_H_
#define N5110_LCD_N5110_LCD_DEFS_H_

#define N5110_WIDTH_PIXELS 		(84)
#define N5110_HEIGHT_PIXELS 	(48)

#define N5110_X_ADDR_LENGTH 	(84)
#define N5110_Y_ADDR_LENGTH 	(6)
#define N5110_TOTAL_ADDR_SIZE 	(N5110_X_ADDR_LENGTH*N5110_Y_ADDR_LENGTH)

// INSTRUCTIONS (FS_EXTENDED_INST_SET = 0 or 1)
#define N5110_NOP 	(0)
#define N5110_FS 	(1<<5)

// Normal Instructions (FS_EXTENDED_INST_SET = 0)
#define N5110_DC 	(1<<3)
#define N5110_SETY	(1<<6)
#define N5110_SETX	(1<<7)

// Extended Instructions (FS_EXTENDED_INST_SET = 1)
#define N5110_TC		(1<<2)
#define N5110_BS		(1<<4)
#define N5110_SETVOP	(1<<7)

// Instruction bits
#define N5110_FS_POWER_DOWN 		(1<<2)
#define N5110_FS_VERTICAL_ADDR 		(1<<1)
#define N5110_FS_EXTENDED_INST_SET 	(1)

#define N5110_DC_DISPLAY_BLANK			(0)
#define N5110_DC_NORMAL_MODE			(1<<2)
#define N5110_DC_ALL_DISP_SEG_ON		(1)
#define N5110_DC_INVERSE_VIDEO_MODE		((1<<2) | 1)

#endif /* N5110_LCD_N5110_LCD_DEFS_H_ */
