/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdlib.h"
#include "stddef.h"
#include "string.h"
#include "gps.h"


#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "N5110_LCD/N5110_lcd.h"


#include "Lora/sx1276/sx1276.h"
#include "Lora/utilities_conf.h"
#include "Lora/Utilities/timeServer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;
States_t State = RX;

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

#define BUFFER_SIZE                                 6       	// Define the payload size here,
uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

GPS_t* gpsHandle;

#ifndef UARTDMA_BUFFER_SIZE
	#define UARTDMA_BUFFER_SIZE 512
#endif
uint8_t GPS_buffer[UARTDMA_BUFFER_SIZE];
uint8_t DMA_buffer1[UARTDMA_BUFFER_SIZE];
uint8_t DMA_buffer2[UARTDMA_BUFFER_SIZE];
uint8_t dma_halfcmplt_flag = 0;
uint8_t uart_data_received_flag = 0;
uint16_t gps_buffer_pointer = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		HAL_GPIO_TogglePin(LD4_RED_GPIO_Port, LD4_RED_Pin);
		memcpy(&DMA_buffer2[UARTDMA_BUFFER_SIZE/2], &DMA_buffer1[UARTDMA_BUFFER_SIZE/2], UARTDMA_BUFFER_SIZE/2);
		memset(&DMA_buffer1[UARTDMA_BUFFER_SIZE/2], 0, UARTDMA_BUFFER_SIZE/2);
		dma_halfcmplt_flag = 0;

		memcpy(GPS_buffer, DMA_buffer2, UARTDMA_BUFFER_SIZE);
		uart_data_received_flag = 1;
		HAL_GPIO_TogglePin(LD4_RED_GPIO_Port, LD4_RED_Pin);
	}
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		HAL_GPIO_TogglePin(LD3_BLUE_GPIO_Port, LD3_BLUE_Pin);
		memcpy(DMA_buffer2, DMA_buffer1, UARTDMA_BUFFER_SIZE/2);
		memset(DMA_buffer1, 0, UARTDMA_BUFFER_SIZE/2);
		if (DMA_buffer2[0] == 0)
		{
			dma_halfcmplt_flag = 1;
		}
		dma_halfcmplt_flag = 1;
		HAL_GPIO_TogglePin(LD3_BLUE_GPIO_Port, LD3_BLUE_Pin);

	}
}


void GPIO_ResetPin(void* port, uint32_t pin)
{
	HAL_GPIO_WritePin(port, pin, 0);
}
void GPIO_SetPin(void* port, uint32_t pin)
{
	HAL_GPIO_WritePin(port, pin, 1);
}
void SPI_Transmit(uint8_t* data, uint32_t length)
{
	HAL_SPI_Transmit(&hspi2, data, length, 10);
}
void SPI_Transmit_DMA(uint8_t* data, uint32_t length)
{
	HAL_SPI_Transmit_DMA(&hspi2, data, length);
}

void PRINTF(const char *strFormat, ...)
{
	static char buf[256];
	va_list vaArgs;
	va_start( vaArgs, strFormat);
	uint16_t bufSize=vsnprintf(buf,sizeof(buf),strFormat, vaArgs);
	va_end(vaArgs);

	HAL_UART_Transmit(&huart1, (uint8_t*)buf, bufSize, 1000);
}

int8_t RssiValue = 0;
int8_t SnrValue = 0;
int LORA_SPREADING_FACTOR = 12;
static  TimerEvent_t timerLed;

void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
    PRINTF("OnTxDone\n\r");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;

    PRINTF("OnRxDone\n\r");
    PRINTF("RssiValue=%d dBm, SnrValue=%d\n\r,   %d", rssi, snr,LORA_SPREADING_FACTOR);
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;

    PRINTF("OnTxTimeout\n\r");
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    PRINTF("OnRxTimeout\n\r,  %d",LORA_SPREADING_FACTOR);
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    PRINTF("OnRxError\n\r");
}

static void OnledEvent( void* context )
{
    HAL_GPIO_TogglePin(LD4_RED_GPIO_Port, LD4_RED_Pin);
    HAL_GPIO_TogglePin(LD2_RED_GPIO_Port, LD2_RED_Pin);
    HAL_GPIO_TogglePin(LD1_GREEN_GPIO_Port, LD1_GREEN_Pin);
    HAL_GPIO_TogglePin(LD3_BLUE_GPIO_Port, LD3_BLUE_Pin);

  TimerStart(&timerLed );
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	bool isMaster = true;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

while(0)
{

	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_15, 0 );
	HAL_Delay(20);
	HAL_GPIO_WritePin( GPIOA, GPIO_PIN_15, 1 );
	HAL_Delay(20);
	}


static RadioEvents_t RadioEvents;
// Radio initialization
RadioEvents.TxDone = OnTxDone;
RadioEvents.RxDone = OnRxDone;
RadioEvents.TxTimeout = OnTxTimeout;
RadioEvents.RxTimeout = OnRxTimeout;
RadioEvents.RxError = OnRxError;

Radio.IoInit( );

/* Led Timers*/
TimerInit(&timerLed, OnledEvent);
#define LED_PERIOD_MS               				200
TimerSetValue( &timerLed, LED_PERIOD_MS);

TimerStart(&timerLed );

Radio.Init( &RadioEvents );

	static uint8_t rxData[2];
	static uint8_t txData[2] = { 0xFF, 0x0};

	char *regstext = "SX1276 registers:\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*)regstext, strlen(regstext), 1000);
	for (uint16_t i=0; i<=0x70 ; i++)
	{
		if (i%16==0)
		{
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 1000);
		}

		//config =  SX1276Read( i );

		txData[0] = i & 0x7F;
		HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 0 );
		HAL_SPI_TransmitReceive( &hspi1, txData, rxData, 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin( RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, 1 );

		char text[8];
		snprintf (text, sizeof(text), "%02X ", rxData[1]);
		HAL_UART_Transmit(&huart1, (uint8_t*)text, 3, 1000);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n_________\r\n", 13, 1000);



#define RF_FREQUENCY                                868000000 // Hz
#define RX_TIMEOUT_VALUE                            10000
#define LED_PERIOD_MS               				200
#define TX_OUTPUT_POWER                             20        // dBm
#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
															  //  1: 250 kHz,
															  //  2: 500 kHz,
															  //  3: Reserved]
	//#define LORA_SPREADING_FACTOR                       10      // [SF7..SF12]
	#define LORA_CODINGRATE                             4         // [1: 4/5,
																  //  2: 4/6,
																  //  3: 4/7,
																  //  4: 4/8]
	#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
	#define LORA_SYMBOL_TIMEOUT                         100       // Symbols
	#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
	#define LORA_IQ_INVERSION_ON                        false


    Radio.SetMaxPayloadLength(MODEM_LORA, 6);

    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    								LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.Rx( RX_TIMEOUT_VALUE );


    while( 1 )
    {
        switch( State )
        {
        case RX:
            if( isMaster == true )
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PongMsg, 4 ) == 0 )
                    {
                        TimerStop(&timerLed );


                        HAL_GPIO_WritePin(LD4_RED_GPIO_Port, LD4_RED_Pin, 0);
                        HAL_GPIO_WritePin(LD1_GREEN_GPIO_Port, LD1_GREEN_Pin, 0);
                        HAL_GPIO_WritePin(LD3_BLUE_GPIO_Port, LD3_BLUE_Pin, 0);
                        // Indicates on a LED that the received frame is a PONG
                        HAL_GPIO_TogglePin(LD2_RED_GPIO_Port, LD2_RED_Pin);

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for(int i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                    	HAL_UART_Transmit(&huart1, (uint8_t*)"...PING\n\r", 9, 1000);
                        //PRINTF("...PING\n\r");

                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );
                    }
                    else if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    { // A master already exists then become a slave
                        isMaster = false;
                        //GpioWrite( &Led2, 1 ); // Set LED off
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            else
            {
                if( BufferSize > 0 )
                {
                    if( strncmp( ( const char* )Buffer, ( const char* )PingMsg, 4 ) == 0 )
                    {
                        // Indicates on a LED that the received frame is a PING
                        TimerStop(&timerLed );
                        HAL_GPIO_WritePin(LD4_RED_GPIO_Port, LD4_RED_Pin, 0);
                        HAL_GPIO_WritePin(LD2_RED_GPIO_Port, LD2_RED_Pin, 0);
                        HAL_GPIO_WritePin(LD1_GREEN_GPIO_Port, LD1_GREEN_Pin, 0);
                        HAL_GPIO_WritePin(LD3_BLUE_GPIO_Port, LD3_BLUE_Pin, 0);

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for(int i = 4; i < BufferSize; i++ )
                        {
                            Buffer[i] = i - 4;
                        }
                        DelayMs( 1 );
                        Radio.Send( Buffer, BufferSize );

                    	HAL_UART_Transmit(&huart1, (uint8_t*)"...PONG\n\r", 9, 1000);
                        //PRINTF("...PONG\n\r");
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx( RX_TIMEOUT_VALUE );
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            //GpioWrite( &Led2, GpioRead( &Led2 ) ^ 1 );
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            if( isMaster == true )
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for(int i = 4; i < BufferSize; i++ )
                {
                    Buffer[i] = i - 4;
                }
                DelayMs( 1 );
                Radio.Send( Buffer, BufferSize );
            }
            else
            {
                Radio.Rx( RX_TIMEOUT_VALUE );
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx( RX_TIMEOUT_VALUE );
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            // Set low power
            break;
        }

        DISABLE_IRQ( );
        /* if an interupt has occured after __disable_irq, it is kept pending
         * and cortex will not enter low power anyway  */
        if (State == LOWPOWER)
        {
        }
        ENABLE_IRQ( );
    }


  AT_N5110_LCD_handle *lcdHandle = AT_N5110LCD_open();
  lcdHandle->functions.delay_ms = HAL_Delay;
  lcdHandle->functions.get_time_ms = HAL_GetTick;
  lcdHandle->functions.reset_gpio = GPIO_ResetPin;
  lcdHandle->functions.set_gpio = GPIO_SetPin;
  lcdHandle->functions.spi_transmit = SPI_Transmit;
  lcdHandle->functions.spi_transmit_dma = SPI_Transmit_DMA;
  lcdHandle->pins.DCPort = N5110_DATA_nCMD_GPIO_Port;
  lcdHandle->pins.DCPin = N5110_DATA_nCMD_Pin;
  lcdHandle->pins.RstPort = N5110_nRST_GPIO_Port;
  lcdHandle->pins.RstPin = N5110_nRST_Pin;
  lcdHandle->pins.csPort = N5110_nCS_GPIO_Port;
  lcdHandle->pins.csPin = N5110_nCS_Pin;
  AT_N5110LCD_init(lcdHandle);


  AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)"FixTakanAt:", 0, 8);
  AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)"Latitude:", 0, 16);
  AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)"Longitude:", 0, 24);
  AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)"Altitude:", 0, 32);
  AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)"Satellites:", 0, 40);
  //AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)"RSSI: ??", 0, 6);
	  AT_N5110LCD_update_display(lcdHandle);

  gpsHandle = gps_open(GPS_DATA_TYPE_NMEA);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, DMA_buffer1, UARTDMA_BUFFER_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  char text[16];
		if (uart_data_received_flag)
		{
			uart_data_received_flag = 0;
			gps_set_input_buffer(gpsHandle, GPS_buffer, UARTDMA_BUFFER_SIZE);

			snprintf(text, sizeof(text), "%d:%d:%d"
					, gpsHandle->gpgga.fixTakenAt.Hour
					, gpsHandle->gpgga.fixTakenAt.Minute
					, gpsHandle->gpgga.fixTakenAt.Second);
			AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)text, 50, 8);
			snprintf(text, sizeof(text), "%f", gpsHandle->gpgga.latitude);
			AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)text, 50, 16);
			snprintf(text, sizeof(text), "%f", gpsHandle->gpgga.longitude);
			AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)text, 50, 24);
			snprintf(text, sizeof(text), "%f", gpsHandle->gpgga.altitude);
			AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)text, 50, 32);
			snprintf(text, sizeof(text), "%d", gpsHandle->gpgga.numSatellites);
			AT_N5110LCD_print_string(lcdHandle, &Font8, (uint8_t*)text, 50, 40);

			AT_N5110LCD_update_display(lcdHandle);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /**Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.SynchPrediv = 1023;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /**Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enable the Alarm A 
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */


  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_GREEN_Pin|LD3_BLUE_Pin|LD4_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RADIO_TCXO_VCC_Pin|N5110_nCS_Pin|RADIO_ANT_SWITCH_RX_Pin|N5110_DATA_nCMD_Pin 
                          |N5110_nRST_Pin|LD2_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RADIO_ANT_SWITCH_TX_BOOST_Pin|RADIO_RESET_Pin|RADIO_ANT_SWITCH_TX_RFO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RADIO_NSS_Pin RADIO_TCXO_VCC_Pin RADIO_ANT_SWITCH_RX_Pin */
  GPIO_InitStruct.Pin = RADIO_NSS_Pin|RADIO_TCXO_VCC_Pin|RADIO_ANT_SWITCH_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_GREEN_Pin LD3_BLUE_Pin LD4_RED_Pin */
  GPIO_InitStruct.Pin = LD1_GREEN_Pin|LD3_BLUE_Pin|LD4_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_DIO_0_Pin RADIO_DIO_1_Pin RADIO_DIO_2_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_0_Pin|RADIO_DIO_1_Pin|RADIO_DIO_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RADIO_DIO_3_Pin */
  GPIO_InitStruct.Pin = RADIO_DIO_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RADIO_DIO_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RADIO_ANT_SWITCH_TX_BOOST_Pin RADIO_RESET_Pin RADIO_ANT_SWITCH_TX_RFO_Pin */
  GPIO_InitStruct.Pin = RADIO_ANT_SWITCH_TX_BOOST_Pin|RADIO_RESET_Pin|RADIO_ANT_SWITCH_TX_RFO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : N5110_nCS_Pin N5110_DATA_nCMD_Pin N5110_nRST_Pin LD2_RED_Pin */
  GPIO_InitStruct.Pin = N5110_nCS_Pin|N5110_DATA_nCMD_Pin|N5110_nRST_Pin|LD2_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
