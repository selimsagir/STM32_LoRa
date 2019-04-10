/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
/**
  *******************************************************************************
  * @file    mlm32l07x01.c
  * @author  MCD Application Team
  * @brief   driver LoRa module murata cmwx1zzabz-078
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "hw.h"
#include "radio.h"
#include "sx1276.h"
#include "mlm32l07x01.h"
static GpioIrqHandler *GpioIrq[16] = { NULL };


#define IRQ_HIGH_PRIORITY  0

#define TCXO_ON()   HAL_GPIO_WritePin( RADIO_TCXO_VCC_GPIO_Port, RADIO_TCXO_VCC_Pin, 1)

#define TCXO_OFF()  HAL_GPIO_WritePin( RADIO_TCXO_VCC_GPIO_Port, RADIO_TCXO_VCC_Pin, 0)

static void SX1276AntSwInit( void );

static void SX1276AntSwDeInit( void );

void SX1276SetXO( uint8_t state );

uint32_t SX1276GetWakeTime( void );

void SX1276IoIrqInit( DioIrqHandler **irqHandlers );

uint8_t SX1276GetPaSelect( uint8_t power );

void SX1276SetAntSwLowPower( bool status );

void SX1276SetAntSw( uint8_t opMode );
/*!
 * \brief Controls the antena switch if necessary.
 *
 * \remark see errata note
 *
 * \param [IN] opMode Current radio operating mode
 */
static LoRaBoardCallback_t BoardCallbacks = { SX1276SetXO,
                                              SX1276GetWakeTime,
                                              SX1276IoIrqInit,
                                              SX1276SetRfTxPower,
                                              SX1276SetAntSwLowPower,
                                              SX1276SetAntSw};

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276IoInit,
    SX1276IoDeInit,
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime
};

uint32_t SX1276GetWakeTime( void )
{
  return  BOARD_WAKEUP_TIME;
}

void SX1276SetXO( uint8_t state )
{

  if (state == SET )
  {
    TCXO_ON(); 
    
    DelayMs( BOARD_WAKEUP_TIME ); //start up time of TCXO
  }
  else
  {
    TCXO_OFF(); 
  }
}
void SX1276IoInit( void )
{
  SX1276BoardInit( &BoardCallbacks );
}

void SX1276IoDeInit( void )
{
}

static uint8_t HW_GPIO_GetBitPos(uint16_t GPIO_Pin)
{
  uint8_t PinPos=0;

  if ( ( GPIO_Pin & 0xFF00 ) != 0) { PinPos |= 0x8; }
  if ( ( GPIO_Pin & 0xF0F0 ) != 0) { PinPos |= 0x4; }
  if ( ( GPIO_Pin & 0xCCCC ) != 0) { PinPos |= 0x2; }
  if ( ( GPIO_Pin & 0xAAAA ) != 0) { PinPos |= 0x1; }

  return PinPos;
}

IRQn_Type MSP_GetIRQn( uint16_t GPIO_Pin)
{
  switch( GPIO_Pin )
  {
    case GPIO_PIN_0:
    case GPIO_PIN_1:  return EXTI0_1_IRQn;
    case GPIO_PIN_2:
    case GPIO_PIN_3:  return EXTI2_3_IRQn;
    case GPIO_PIN_4:
    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:
    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15:
    default: return EXTI4_15_IRQn;
  }
}
void HW_GPIO_SetIrq( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t prio,  GpioIrqHandler *irqHandler )
{
  IRQn_Type IRQnb;

  uint32_t BitPos = HW_GPIO_GetBitPos( GPIO_Pin ) ;

  if ( irqHandler != NULL)
  {
    GpioIrq[ BitPos ] = irqHandler;

    IRQnb = MSP_GetIRQn( GPIO_Pin );

    HAL_NVIC_SetPriority( IRQnb , prio, 0);

    HAL_NVIC_EnableIRQ( IRQnb );
  }
  else
  {
    GpioIrq[ BitPos ] = NULL;
  }
}


void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{

  HW_GPIO_SetIrq( GPIOB, GPIO_PIN_4, IRQ_HIGH_PRIORITY, irqHandlers[0] );
  HW_GPIO_SetIrq( GPIOB, GPIO_PIN_1, IRQ_HIGH_PRIORITY, irqHandlers[1] );
  HW_GPIO_SetIrq( GPIOB, GPIO_PIN_0, IRQ_HIGH_PRIORITY, irqHandlers[2] );
  HW_GPIO_SetIrq( GPIOB, GPIO_PIN_13, IRQ_HIGH_PRIORITY, irqHandlers[3] );

}


static void SX1276AntSwInit( void )
{
  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, 0);

  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_TX_BOOST_GPIO_Port, RADIO_ANT_SWITCH_TX_BOOST_Pin, 0);

  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_TX_RFO_GPIO_Port, RADIO_ANT_SWITCH_TX_RFO_Pin, 0);
}

static void SX1276AntSwDeInit( void )
{
  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, 1);

  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_TX_BOOST_GPIO_Port, RADIO_ANT_SWITCH_TX_BOOST_Pin, 1);

  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_TX_RFO_GPIO_Port, RADIO_ANT_SWITCH_TX_RFO_Pin, 1);
}



void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( power );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( uint8_t power )
{
    if (power >14)
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( status == false )
    {
      SX1276AntSwInit( );
    }
    else 
    {
      SX1276AntSwDeInit( );
    }
}

void SX1276SetAntSw( uint8_t opMode )
{
 uint8_t paConfig =  SX1276Read( REG_PACONFIG );
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
      if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
      {
    	  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_TX_BOOST_GPIO_Port, RADIO_ANT_SWITCH_TX_BOOST_Pin, 1 );
      }
      else
      {
    	  HAL_GPIO_WritePin( RADIO_ANT_SWITCH_TX_RFO_GPIO_Port, RADIO_ANT_SWITCH_TX_RFO_Pin, 1 );
      }
      SX1276.RxTx = 1;
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
     SX1276.RxTx = 0;
     HAL_GPIO_WritePin(RADIO_ANT_SWITCH_RX_GPIO_Port, RADIO_ANT_SWITCH_RX_Pin, 1 );
     break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
