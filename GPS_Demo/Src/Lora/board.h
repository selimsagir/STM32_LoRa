/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stm32l072xx.h"
#include "stm32l0xx_hal.h"
#include "utilities.h"
#include "gpio.h"
#include "spi.h"
#include "radio/radio.h"
#include "radio/sx1276/sx1276.h"
#include "sx1276-board.h"
/*
#include "i2c.h"
#include "adc.h"
#include "timer.h"
#include "delay.h"
#include "uart.h"
#include "mag3110.h"
#include "mma8451.h"
#include "mpl3115.h"
#include "sx9500.h"
#include "gps.h"
#include "gps-board.h"
#include "rtc-board.h"
#include "uart-board.h"*/

#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

/*!
 * Define indicating if an external IO expander is to be used
 */
#define BOARD_IOE_EXT

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

/*!
 * Board IO Extender pins definitions
 */
#define IRQ_MPL3115                                 IOE_0
#define IRQ_MAG3110                                 IOE_1
#define GPS_POWER_ON                                IOE_2
#define RADIO_PUSH_BUTTON                           IOE_3
#define BOARD_POWER_DOWN                            IOE_4
#define SPARE_IO_EXT_5                              IOE_5
#define SPARE_IO_EXT_6                              IOE_6
#define SPARE_IO_EXT_7                              IOE_7
#define N_IRQ_SX9500                                IOE_8
#define IRQ_1_MMA8451                               IOE_9
#define IRQ_2_MMA8451                               IOE_10
#define TX_EN_SX9500                                IOE_11
#define LED_1                                       IOE_12
#define LED_2                                       IOE_13
#define LED_3                                       IOE_14
#define LED_4                                       IOE_15

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 RADIO_RESET_Pin

#define RADIO_MOSI                                  RADIO_MOSI_Pin
#define RADIO_MISO                                  RADIO_MISO_Pin
#define RADIO_SCLK                                  RADIO_SCLK_Pin
#define RADIO_NSS                                   RADIO_NSS_Pin

#define RADIO_DIO_0                                 RADIO_DIO_0_Pin
#define RADIO_DIO_1                                 RADIO_DIO_1_Pin
#define RADIO_DIO_2                                 RADIO_DIO_2_Pin
#define RADIO_DIO_3                                 RADIO_DIO_3_Pin
/*
#define RADIO_ANT_SWITCH_HF                         PA_0
#define RADIO_ANT_SWITCH_LF                         PA_1
*/
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define USB_DM                                      PA_11
#define USB_DP                                      PA_12

#define I2C_SCL                                     PB_6
#define I2C_SDA                                     PB_7

#define BOOT_1                                      PB_2
    
#define GPS_PPS                                     PB_1
#define UART_TX                                     PA_9
#define UART_RX                                     PA_10

#define DC_DC_EN                                    PB_8
#define BAT_LEVEL                                   PB_0
#define WKUP1                                       PA_8
#define USB_ON                                      PA_2

#define RF_RXTX                                     PA_3

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define TEST_POINT1                                 PB_12
#define TEST_POINT2                                 PB_13
#define TEST_POINT3                                 PB_14
#define TEST_POINT4                                 PB_15

#define PIN_NC                                      PB_5

/*!
 * \brief Initializes the target board peripherals.
 */
void BoardInitMcu( void );

/*!
 * \brief Initializes the boards peripherals.
 */
void BoardInitPeriph( void );

/*!
 * \brief De-initializes the target board peripherals to decrease power
 *        consumption.
 */
void BoardDeInitMcu( void );

/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );

#endif // __BOARD_H__
