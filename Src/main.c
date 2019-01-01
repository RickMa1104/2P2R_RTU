/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "rtu.h"
#define REG_INPUT_START       0
#define REG_INPUT_NREGS       8
#define REG_HOLDING_START     0
#define REG_HOLDING_NREGS     8
#define REG_COILS_START       0 
#define REG_COILS_SIZE        16       
#define REG_DISCRETE_START    0
#define REG_DISCRETE_SIZE     16
#define  MAX_LEN              256
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum
{
    MB_ENOERR,                  /*!< no error. */
    MB_ENOREG,                  /*!< illegal register address. */
    MB_EINVAL,                  /*!< illegal argument. */
    MB_EPORTERR,                /*!< porting layer error. */
    MB_ENORES,                  /*!< insufficient resources. */
    MB_EIO,                     /*!< I/O error. */
    MB_EILLSTATE,               /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT                /*!< timeout error occurred. */
} eMBErrorCode;

typedef enum
{
    MB_REG_READ,                /*!< Read register values and pass to protocol stack. */
    MB_REG_WRITE                /*!< Update register values. */
} eMBRegisterMode;

static uint16_t     usRegInputStart = REG_INPUT_START;
static uint16_t     usRegInputBuf[REG_INPUT_NREGS];
static uint16_t     usRegHoldingStart = REG_HOLDING_START;
static uint16_t     usRegHoldingBuf[REG_HOLDING_NREGS];
static uint8_t      ucRegCoilsBuf[(REG_COILS_SIZE - 1)/8 + 1];
static uint8_t      ucRegDiscreteBuf[(REG_DISCRETE_SIZE - 1 )/8 + 1];
uint8_t           apc240RevDate[MAX_LEN];
extern RTU        rtu;
uint32_t          converValue[6];                     // adc 转换值
uint32_t          counts[2];                          //流量计数
volatile uint8_t           tempFlag = 0;
uint8_t           coilFlag3 = 5, coilFlag4 = 5;
uint8_t           apc240RecLen; 
uint8_t           backData[128];
uint8_t           sendData[128];
uint8_t           tempData[128];
uint8_t           set[8];
float value; 
float vdda;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void InitInputReg(void);
void InitHoldingReg(void);
void OperationHoldingReg(uint8_t * pucRegBuffer, uint16_t usAddress);
void UpdateInputReg(void);
void UpdateHoldingReg(void);
uint8_t xMBUtilGetBits( uint8_t * ucByteBuf, uint16_t usBitOffset, uint8_t ucNBits );
eMBErrorCode eMBRegCoilsCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils,
               eMBRegisterMode eMode );
eMBErrorCode  eMBRegDiscreteCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete );
eMBErrorCode  eMBRegInputCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs );
eMBErrorCode  eMBRegHoldingCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs,
                 eMBRegisterMode eMode );
void eMBPoll( uint8_t* data, uint8_t lenght );								 
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_LPUART1_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&hlpuart1, apc240RevDate, MAX_LEN);
	__HAL_UART_ENABLE_IT(&hlpuart1,UART_IT_IDLE);
	HAL_PWREx_EnableUltraLowPower();
	HAL_ADCEx_Calibration_Start(&hadc, 0);
	HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AD_POWER_EN_GPIO_Port, AD_POWER_EN_Pin, GPIO_PIN_RESET);
	RtuDateInit();
	InitInputReg();
	InitHoldingReg();
	Apc240SetFre(rtu.apc240_Channel);
  HAL_ADCEx_EnableVREFINT();
	UpdateInputReg();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		  HAL_Delay(10);	
			uint8_t  lenght = apc240RecLen;
		  if(apc240RecLen >= 128)
			{
				apc240RecLen = 0;
				lenght = 0;
			}
			for(int i = 0; i < lenght; i ++)
			{
				tempData[i] = apc240RevDate[i];
			}
			ClearRcvBuff();
			eMBPoll( tempData, lenght);
			Apc240ModeSet(Apc240Mode2);
			
			if(coilFlag3 == 1)
			{
				coilFlag3=2;
				HAL_Delay(5000);
				UpdateInputReg();	
				HAL_Delay(500);
				UpdateHoldingReg();	
			}
			else if(coilFlag3 == 0)
			{
				coilFlag3=3;
				HAL_Delay(5000);
				UpdateInputReg();	
				HAL_Delay(500);
				UpdateHoldingReg();	
			}
			if(coilFlag4 == 1)
			{
				coilFlag4=2;
				HAL_Delay(5000);
				UpdateInputReg();
				HAL_Delay(500);
				UpdateHoldingReg();	
			}
			else if(coilFlag4 == 0)
			{
				coilFlag4=3;
				HAL_Delay(5000);
				UpdateInputReg();	
				HAL_Delay(500);
				UpdateHoldingReg();	
			}
			/*
			异常处理
			*/
			if(rtu.rtuId != ReadDateEeprom(IdAddress))
			{
					HAL_NVIC_SystemReset();
			}
			__HAL_DMA_DISABLE(&hdma_lpuart1_rx);
		  hdma_lpuart1_rx.Instance->CNDTR = MAX_LEN;
	  	__HAL_DMA_ENABLE(&hdma_lpuart1_rx);		
			///

			if(coilFlag3 == 0 && coilFlag4 == 0)
			{
				HAL_SuspendTick();
				HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
				SystemClock_Config();	
			}
//			
			
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* LPUART1 init function */
void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 38400;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&hlpuart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, AD_POWER_EN_Pin|BAT_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|FM3Z_Pin|FM2F_Pin|FM2Z_Pin 
                          |FM1F_Pin|FM1Z_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FMCF_Pin|FMCZ_Pin|FM4F_Pin|FM4Z_Pin 
                          |FM3F_Pin|PWM_OUT_Pin|PWM_EN_Pin|SETB_Pin 
                          |SETA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : AD_POWER_EN_Pin BAT_EN_Pin */
  GPIO_InitStruct.Pin = AD_POWER_EN_Pin|BAT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RING_Pin */
  GPIO_InitStruct.Pin = RING_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RING_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin FM3Z_Pin FM2F_Pin FM2Z_Pin 
                           FM1F_Pin FM1Z_Pin */
  GPIO_InitStruct.Pin = LED_Pin|FM3Z_Pin|FM2F_Pin|FM2Z_Pin 
                          |FM1F_Pin|FM1Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FMCF_Pin FMCZ_Pin FM4F_Pin FM4Z_Pin 
                           FM3F_Pin PWM_EN_Pin SETB_Pin SETA_Pin */
  GPIO_InitStruct.Pin = FMCF_Pin|FMCZ_Pin|FM4F_Pin|FM4Z_Pin 
                          |FM3F_Pin|PWM_EN_Pin|SETB_Pin|SETA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWM_OUT_Pin */
  GPIO_InitStruct.Pin = PWM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PWM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AUX_Pin */
  GPIO_InitStruct.Pin = AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AUX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void
xMBUtilSetBits( uint8_t * ucByteBuf,  uint16_t usBitOffset, uint8_t ucNBits,
                uint8_t ucValue )
{
    uint16_t          usWordBuf;
    uint16_t          usMask;
    uint16_t          usByteOffset;
    uint16_t          usNPreBits;
    uint16_t          usValue = ucValue;

  //  assert( ucNBits <= 8 );
  //  assert( ( size_t )BITS_UCHAR == sizeof( uint8_t ) * 8 );

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = (  uint16_t )( ( usBitOffset ) / 8 );

    /* How many bits precede our bits to set. */
    usNPreBits = (  uint16_t )( usBitOffset - usByteOffset * 8 );

    /* Move bit field into position over bits to set */
    usValue <<= usNPreBits;

    /* Prepare a mask for setting the new bits. */
    usMask = (  uint16_t )( ( 1 << (  uint16_t ) ucNBits ) - 1 );
    usMask <<= usBitOffset - usByteOffset * 8;

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << 8;

    /* Zero out bit field bits and then or value bits into them. */
    usWordBuf = (  uint16_t )( ( usWordBuf & ( ~usMask ) ) | usValue );

    /* move bits back into storage */
    ucByteBuf[usByteOffset] = ( uint8_t )( usWordBuf & 0xFF );
    ucByteBuf[usByteOffset + 1] = ( uint8_t )( usWordBuf >> 8 );
}

uint8_t
xMBUtilGetBits( uint8_t * ucByteBuf, uint16_t usBitOffset, uint8_t ucNBits )
{
    uint8_t          usWordBuf;
    uint8_t          usMask;
    uint8_t          usByteOffset;
    uint8_t          usNPreBits;

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( uint16_t )( ( usBitOffset ) / 8 );

    /* How many bits precede our bits to set. */
    usNPreBits = ( uint16_t )( usBitOffset - usByteOffset * 8 );

    /* Prepare a mask for setting the new bits. */
    usMask = ( uint16_t )( ( 1 << ( uint16_t ) ucNBits ) - 1 );

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << 8;

    /* throw away unneeded bits. */
    usWordBuf >>= usNPreBits;

    /* mask away bits above the requested bitfield. */
    usWordBuf &= usMask;

    return ( uint8_t ) usWordBuf;
}

void eMBPoll( uint8_t* data, uint8_t lenght )
{
		if(usMBCRC16( data, lenght) == 0 && lenght == 0x08)
		{
				if(data[0] == rtu.rtuId && data[1] != 0)
				{
					  uint8_t sendLength = 0;
					  sendData[0] = rtu.rtuId;
					  sendData[1] = data[1];
						uint16_t  startAdress = data[2] <<8 | data[3];
					  uint16_t  dataNumbers  = data[4] <<8 | data[5];
					  uint16_t   crc;
					  eMBRegisterMode eMode;
						if( data[1] == 0x01)
						{
								eMode =  MB_REG_READ;
								if(dataNumbers >= 0x01 && dataNumbers <= 0x07d0)	 
								{
								    if(eMBRegCoilsCB( backData, startAdress, dataNumbers, eMode ) == MB_ENOERR)
										{
											  uint8_t  temp = (dataNumbers - 1)/8; 
										  	sendData[2]= temp + 1;
												for(int i = 0; i < temp +1; i++)
												{
														sendData[3 + i] = backData[temp];
												}
												crc = usMBCRC16(sendData,4 + temp);
                        sendData[4 + temp] = crc;
                        sendData[5 + temp] = crc>>8;
											  sendLength  = 6 + temp;
												Apc240SendWait(sendData, sendLength);
										}
										else
										{
											// 02 error
											sendData[1] = 0x80 + data[1];
											sendData[2] = 0x02;
											crc = usMBCRC16(sendData,3);
                      sendData[3] = crc;
                      sendData[4] = crc>>8;
											sendLength  = 0x05;
											Apc240SendWait(sendData, sendLength);
										}
								}
								else
								{
								//03 error
									  sendData[1] = 0x80 + data[1];
										sendData[2] = 0x02;
										crc = usMBCRC16(sendData,3);
                    sendData[3] = crc;
                    sendData[4] = crc>>8;
									  sendLength  = 0x05;
								  	Apc240SendWait(sendData, sendLength);
								}
						}
						
						else if(data[1] == 0x02)
						{
								if(dataNumbers >= 0x01 && dataNumbers <= 0x07d0)	 
								{
								    if(eMBRegDiscreteCB( backData, startAdress, dataNumbers ) == MB_ENOERR)
										{
												uint8_t  temp = (dataNumbers -1 )/8; 
										  	sendData[2]= temp + 1;
												for(int i = 0; i < temp +1; i++)
												{
														sendData[3 + i] = backData[temp];
												}
												crc = usMBCRC16(sendData,4 + temp);
                        sendData[4 + temp] = crc;
                        sendData[5 + temp] = crc>>8;
											  sendLength  = 6 + temp;
												Apc240SendWait(sendData, sendLength);
										}
										else
										{
											// 02 error
											sendData[1] = 0x80 + data[1];
											sendData[2] = 0x02;
											crc = usMBCRC16(sendData,3);
                      sendData[3] = crc;
                      sendData[4] = crc>>8;
											sendLength  = 0x05;
											Apc240SendWait(sendData, sendLength);
										}
								}
								else
								{
								//03 error
									  sendData[1] = 0x80 + data[1];
										sendData[2] = 0x02;
									  crc = usMBCRC16(sendData,3);
                    sendData[3] = crc;
                    sendData[4] = crc>>8;
									  sendLength  = 0x05;
								    Apc240SendWait(sendData, sendLength);
								}
						}
						
						else if(data[1] == 0x03 )
						{
								if(dataNumbers >= 0x01 && dataNumbers <= 0x007d)	 
								{
										eMode = MB_REG_READ;
								    if(eMBRegHoldingCB( backData, startAdress, dataNumbers, eMode ) == MB_ENOERR)
										{
//														sendData[2] = dataNumbers ;
//													  for(int i = 0; i < dataNumbers; i ++)
//														{
//																sendData[3 + i*2] = backData[i*2] ;
//																sendData[4 + i*2] = backData[i*2+1] ;
//														}
//														crc = usMBCRC16(sendData, 3+ dataNumbers*2);
//                            sendData[3+ dataNumbers*2] = crc;
//                            sendData[4+ dataNumbers*2] = crc>>8;
//											      sendLength  = 5+ dataNumbers*2;
//														Apc240SendWait(sendData, sendLength);
										}
										else
										{
											// 02 error
											sendData[1] = 0x80 + data[1];
										  sendData[2] = 0x02;
											crc = usMBCRC16(sendData,3);
                      sendData[3] = crc;
                      sendData[4] = crc>>8;
											sendLength  = 0x05;
											Apc240SendWait(sendData, sendLength);
										}
								}
								else
								{
								//03 error
									  sendData[1] = 0x80 + data[1];
										sendData[2] = 0x03;
								  	crc = usMBCRC16(sendData,3);
                    sendData[3] = crc;
                    sendData[4] = crc>>8;
									  sendLength  = 0x05;
								  	Apc240SendWait(sendData, sendLength);
								}
						}
						
						else if(data[1] == 0x04)
						{
								if(dataNumbers >= 0x01 && dataNumbers <= 0x007d)	 
								{
								    if(eMBRegInputCB( backData, startAdress, dataNumbers ) == MB_ENOERR)
										{
//												sendData[2] = dataNumbers ;
//											  for(int i = 0; i < dataNumbers; i ++)
//													{
//															sendData[3 + i*2] = backData[i*2] ;
//															sendData[4 + i*2] = backData[i*2+1];
//													}
//													crc = usMBCRC16(sendData, 3+ dataNumbers*2);
//                          sendData[3+ dataNumbers*2] = crc;
//                          sendData[4+ dataNumbers*2] = crc>>8;
//											    sendLength  = 5+ dataNumbers*2;
//													Apc240SendWait(sendData, sendLength);
										}
										else
										{
											// 02 error
											sendData[1] = 0x80 + data[1];
										  sendData[2] = 0x02;
											crc = usMBCRC16(sendData,3);
                      sendData[3] = crc;
                      sendData[4] = crc>>8;
											sendLength  = 0x05;
											Apc240SendWait(sendData, sendLength);
										}
								}
								else
								{
								//03 error
									  sendData[1] = 0x80 + data[1];
										sendData[2] = 0x03;
										crc = usMBCRC16(sendData,3);
                    sendData[3] = crc;
                    sendData[4] = crc>>8;
									  sendLength  = 0x05;
								  	Apc240SendWait(sendData, sendLength);
								}
						}
						
						else if(data[1] == 0x05)
						{
							Apc240SendWait(data,8);
						  eMode =  MB_REG_WRITE;
								if(dataNumbers == 0x0000 || dataNumbers == 0xff00)	 
								{
										if(dataNumbers == 0)
										{
											set[0] = 0;
										}
										else
										{
											set[0] = 1;
										}
								    if(eMBRegCoilsCB( set, startAdress, 1, eMode ) == MB_ENOERR)
										{
												HAL_Delay(15);
										}
										else
										{
											// 02 error
											sendData[1] = 0x80 + data[1];
										  sendData[2] = 0x02;
											crc = usMBCRC16(sendData,3);
                      sendData[3] = crc;
                      sendData[4] = crc>>8;
											sendLength  = 0x05;
											Apc240SendWait(sendData, sendLength);
										}
								}
								else
								{
								//03 error
									  sendData[1] = 0x80 + data[1];
										sendData[2] = 0x03;
										crc = usMBCRC16(sendData,3);
                    sendData[3] = crc;
                    sendData[4] = crc>>8;
									  sendLength  = 0x05;
									  Apc240SendWait(sendData, sendLength);
								}
						}
						else if(data[1] == 0x06)
						{
										eMode = MB_REG_WRITE;
						      	set[0] = dataNumbers >> 8;
							      set[1] = dataNumbers & 0xff ;
								    if(eMBRegHoldingCB( set, startAdress, 1, eMode ) == MB_ENOERR)
										{
												 Apc240SendWait(data,8);
										}
										else
										{
											// 02 error
											sendData[1] = 0x80 + data[1];
										  sendData[2] = 0x02;
											crc = usMBCRC16(sendData,3);
                      sendData[3] = crc;
                      sendData[4] = crc>>8;
											sendLength  = 0x05;
											Apc240SendWait(sendData, sendLength);
										}
							
						}
						else
						{
								// 01 error
								  sendData[1] = 0x80 + data[1];
									sendData[2] = 0x01;
									crc = usMBCRC16(sendData,3);
                  sendData[3] = crc;
                  sendData[4] = crc>>8;
							    sendLength  = 0x05;
							    Apc240SendWait(sendData, sendLength);
						}
						memset((void*)data, 0 , sizeof(uint8_t) * lenght);
				}
				
				if(rtu.setMode == 1)
				{
						RtuSet(data, lenght);
						
				}
		}
}

eMBErrorCode
eMBRegInputCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
	
	 LedOn();
 UpdateInputReg();
	 if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
		LedOff();
    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNRegs,
                 eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
		LedOn();
 UpdateHoldingReg();
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
							  OperationHoldingReg(pucRegBuffer, usAddress);
            }
						 break;     
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
		LedOff();
    return eStatus; 
}


eMBErrorCode
eMBRegCoilsCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNCoils,
               eMBRegisterMode eMode )
{
   eMBErrorCode eStatus = MB_ENOERR;  
	 int16_t iNCoils = ( int16_t )usNCoils;  
   int16_t usBitOffset; 
   LedOn();	
   if( ( (int16_t)usAddress >= REG_COILS_START ) &&  
      (  usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )  
      {  
          usBitOffset = ( int16_t )( usAddress - REG_COILS_START );  
					switch ( eMode )  
									{   
									case MB_REG_READ:  
									while( iNCoils > 0 )  
									{  
										*pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,  
										( uint8_t )( iNCoils > 8 ? 8 : iNCoils ) );  
										iNCoils -= 8;  
										usBitOffset += 8;  
									}  
									break;  
										 
									case MB_REG_WRITE:  
									while( iNCoils > 0 )  
									{ 
										if(usAddress == 3 || usAddress == 2)
										{
													OperateCoil(* pucRegBuffer, usAddress+1 );									
										}
										xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,  
										( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),  
										*pucRegBuffer++ );  
										iNCoils -= 8;  
									}  
									break;  
									}  
  
		   }  
  else  
			{  
					eStatus = MB_ENOREG;  
			} 
		  LedOff();			
			return eStatus;  
}  



eMBErrorCode
eMBRegDiscreteCB( uint8_t * pucRegBuffer, uint16_t usAddress, uint16_t usNDiscrete )
{
	eMBErrorCode eStatus = MB_ENOERR;
  int16_t iNDiscrete = ( int16_t )usNDiscrete;
	uint16_t usBitOffset;
	ucRegDiscreteBuf[0] = UpdateInput();
	LedOn();
  if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
     ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
		{
			usBitOffset = ( uint16_t )( usAddress - REG_DISCRETE_START );
			while( iNDiscrete > 0 )
				{
					*pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
					( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );
					iNDiscrete -= 8;
					usBitOffset += 8;
				}
		}
	else
		{
			eStatus = MB_ENOREG;
		}
	LedOff();
	return eStatus;

}



void InitInputReg()
{
	usRegInputBuf[0] = rtu.rtuId;
	usRegInputBuf[1] = rtu.apc240_Channel;
	usRegInputBuf[2] = rtu.voltage;
	usRegInputBuf[3] = rtu.temperature;	
}

void InitHoldingReg()
{
//	uint16_t temp1 = 0, temp2 = 0;
//	usRegHoldingBuf[0] = rtu.chargeTime;
//	usRegHoldingBuf[1] = rtu.batChargeTime;
//	temp1 = ReadDateEeprom(LIU1V4);
//	temp1 = temp1 << 8;
//	temp1 = temp1 | ReadDateEeprom(LIU1V3);
//	temp2 = ReadDateEeprom(LIU1V2);
//	temp2 = temp2 << 8;
//	temp2 = temp2 | ReadDateEeprom(LIU1V1);
//	usRegHoldingBuf[2] = temp1;
//	usRegHoldingBuf[3] = temp2;
}

void OperationHoldingReg(uint8_t * pucRegBuffer, uint16_t usAddress)
{
	uint16_t temp;
	temp = *pucRegBuffer++ << 8;
  temp |= *pucRegBuffer++;
	if(usAddress == 1)
	{
		if(temp <= 15)
		{
			rtu.chargeTime = temp;
		}
		else 
		{
			usRegHoldingBuf[0] = rtu.chargeTime;
		}
	}
	else if(usAddress == 2)
	{
		if(temp <= 200)
		{
			rtu.batChargeTime = temp;
		}
		else
		{
			usRegHoldingBuf[1] = rtu.batChargeTime;
		}
	}
}

void UpdateInputReg()
{
	//__HAL_ADC_ENABLE(&hadc);
if(coilFlag3==2||coilFlag3==3)
{
	OutputVoltage(open,1);
	coilFlag3=5;
} 
if(coilFlag4==2||coilFlag4==3)
{
	OutputVoltage(open,2);
	coilFlag4=5;
}
  HAL_GPIO_WritePin(AD_POWER_EN_GPIO_Port, AD_POWER_EN_Pin, GPIO_PIN_SET);
	HAL_ADC_Start_DMA(&hadc, converValue, 6);
	HAL_Delay(400);
	HAL_ADC_Stop_DMA(&hadc);
	HAL_ADC_Start_DMA(&hadc, converValue, 6);
	HAL_Delay(400);
	vdda = *VREFINT_CAL*3.0f/(converValue[3] );
	usRegInputBuf[2] = (uint16_t)(vdda * converValue[2] *3200.f*2/4095);
	usRegInputBuf[3] = (uint16_t)(vdda * converValue[0] *3200.f*(3.3f+6.8f)/3.3f/4095.f);
  usRegInputBuf[4] = (uint16_t)(vdda * converValue[1] *3200.f*(3.3f+6.8f)/3.3f/4095.f);
	rtu.voltage = vdda*(converValue[2] )*2/4095;
	HAL_Delay(200);
	HAL_GPIO_WritePin(AD_POWER_EN_GPIO_Port, AD_POWER_EN_Pin, GPIO_PIN_RESET);
	HAL_ADC_Stop_DMA(&hadc);
	OutputVoltage(close,1);
  OutputVoltage(close,2);
	  uint16_t crc;
		uint8_t dataNumbers=5;
		uint8_t sendLength = 0;
		sendData[0] = rtu.rtuId;
		sendData[1] = 0x04;
		sendData[2] = dataNumbers ;
		for(int i = 0; i < 10; i ++)
		{
				sendData[3 + i*2] = usRegInputBuf[i]>>8;
				sendData[4 + i*2] = usRegInputBuf[i]& 0xFF;
		}
		crc = usMBCRC16(sendData, 3+ dataNumbers*2);
		sendData[3+ dataNumbers*2] = crc;
		sendData[4+ dataNumbers*2] = crc>>8;
		sendLength  = 5+ dataNumbers*2;
		Apc240ModeSet(Apc240Mode1);
		HAL_Delay(50);
		Apc240Send(sendData,sendLength);
	// __HAL_ADC_DISABLE(&hadc);
}

void UpdateHoldingReg()
{
//		usRegHoldingBuf[3] = (uint16_t)(counts[0]>> 16);
//		usRegHoldingBuf[2] =  (uint16_t) counts[0];
//		usRegHoldingBuf[5] = (uint16_t)(counts[1] >> 16);
//		usRegHoldingBuf[4] =  (uint16_t) counts[1];
	  uint16_t crc;
		uint8_t dataNumbers=6;
		uint8_t sendLength = 0;
		sendData[0] = rtu.rtuId;
		sendData[1] = 0x03;
		sendData[2] = dataNumbers;
		for(int i = 0; i < 10; i ++)
		{
				sendData[3 + i*2] = usRegHoldingBuf[i]>>8;
				sendData[4 + i*2] = usRegHoldingBuf[i]& 0xFF;
		}
		crc = usMBCRC16(sendData, 3+ dataNumbers*2);
		sendData[3+ dataNumbers*2] = crc;
		sendData[4+ dataNumbers*2] = crc>>8;
		sendLength  = 5+ dataNumbers*2;
		Apc240ModeSet(Apc240Mode1);
		HAL_Delay(50);
		Apc240Send(sendData,sendLength);  
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == SW_Pin)
	{
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin) == GPIO_PIN_RESET)
		{
			HAL_Delay(1000);
			if(HAL_GPIO_ReadPin(SW_GPIO_Port,SW_Pin) == GPIO_PIN_RESET)
		  {
				if(rtu.setMode == 0)
				{
					Apc240SetFre(apc240_channel_21);
					rtu.setMode = 1;
					LedOn();
				}
				else
				{
					Apc240SetFre(rtu.apc240_Channel);
					rtu.setMode = 0;
					LedOff();
				}
		}
	}
}
	else if(GPIO_Pin == AUX_Pin)
	{
//		if(rtu.setFreMode == 0)
//		{
//			Apc240ModeSet(Apc240Mode1);
//		}
	}
	
		if(GPIO_Pin == GPIO_PIN_4 || GPIO_Pin == GPIO_PIN_1)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1) == GPIO_PIN_RESET )
		{
			 HAL_Delay(500);
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_RESET)
			{
						// 14 kai
				sendData[3]= 0x09;
			}
			else
			{
					//1 k  4 bk
				sendData[3]= 0x01;
			}
		}
		else
		{
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_RESET)
			{
				//1 bk 4 k	
				sendData[3]= 0x08;	
			}
			else
			{
				//1 bk  4 bk
				sendData[3]= 0x00;
			}
		}
				uint16_t crc;
				sendData[0] = rtu.rtuId;
				sendData[1] = 0x02;
				sendData[2]= 0x01;
				crc=usMBCRC16(sendData,4);
				sendData[5]= crc>>8;
				sendData[4]= crc;	
				Apc240ModeSet(Apc240Mode1);
		    HAL_Delay(20);
				Apc240Send(sendData,6);
		    HAL_Delay(500);
	      UpdateInputReg();
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
