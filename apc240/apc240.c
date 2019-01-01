#include"apc240.h"
typedef uint8_t u8;
extern UART_HandleTypeDef hlpuart1;
char tab[]={0x7f,0x3f,0x1f,0xf,0x7,0x3,0x1,0x0};
RTU rtu;
void Apc240ModeSet(Apc240Mode Mode)
{
    switch (Mode)
            {
									case Apc240Mode1 :HAL_GPIO_WritePin(SETA_GPIO_Port, SETA_Pin, GPIO_PIN_RESET);
																		HAL_GPIO_WritePin(SETB_GPIO_Port, SETB_Pin, GPIO_PIN_RESET);
																		break;
									case Apc240Mode2 :HAL_GPIO_WritePin(SETA_GPIO_Port, SETA_Pin, GPIO_PIN_RESET);
																		HAL_GPIO_WritePin(SETB_GPIO_Port, SETB_Pin, GPIO_PIN_SET);
																		break;
									case Apc240Mode3 :HAL_GPIO_WritePin(SETA_GPIO_Port, SETA_Pin, GPIO_PIN_SET);
																		HAL_GPIO_WritePin(SETB_GPIO_Port, SETB_Pin, GPIO_PIN_RESET);
																		break;
									case Apc240Mode4 :HAL_GPIO_WritePin(SETA_GPIO_Port, SETA_Pin, GPIO_PIN_SET);
																		HAL_GPIO_WritePin(SETB_GPIO_Port, SETB_Pin, GPIO_PIN_SET);
																		break;
            }
}

void Apc240Send(u8 *data,int lenght)
{ 
   HAL_UART_Transmit(&hlpuart1, data ,lenght, 0x01 );
}

void Apc240SendComplete()
{
		while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == GPIO_PIN_RESET)
		{
		
		}
}

void Apc240SendWait(uint8_t * sendData, uint8_t sendLength)
{
		Apc240ModeSet(Apc240Mode1);
		HAL_Delay(5);
		Apc240Send(sendData, sendLength);
		Apc240SendComplete();
}
void UartInit(int baudrate)
{
	hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = baudrate;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&hlpuart1);
}

void Apc240SetFre(Apc240_Channel apc240_channel)
{ 
	u8 fre[22][3]={{0x06,0x87,0xe0},{0x06,0x89,0xd4},{0x06,0x8b,0xc8},{0x06,0x8d,0xbc},{0x06,0x8f,0xb0},{0x06,0x91,0xa4},
                 {0x06,0x93,0x98},{0x06,0x95,0x8c},{0x06,0x97,0x80},{0x06,0x99,0x74},{0x06,0x90,0x5c},{0x06,0x9f,0x50},
                 {0x06,0xa1,0x44},{0x06,0xa3,0x38},{0x06,0xa5,0x2c},{0x06,0xa7,0x20},{0x06,0xa9,0x14},{0x06,0xab,0x08},
                 {0x06,0xac,0xfc},{0x06,0xae,0xf0},{0x06,0xb0,0xe4},{0x06,0x9b,0x68} };
 // u8 apc240read[7]={0xff,0x56,0xae,0x35,0xa9,0x55,0xf0};
	u8 apc240set[15]={0xff,0x56,0xae,0x35,0xa9,0x55,0x90,0x06,0x9b,0x68,0x00,0x07,0x05,0x00,0x06};
	UartInit(9600);
	rtu.setFreMode = 1;
	Apc240ModeSet(Apc240Mode4);
	while(HAL_GPIO_ReadPin(AUX_GPIO_Port, AUX_Pin) == RESET)
	{
	
	}
	HAL_Delay(1000);
  apc240set[7] = fre[apc240_channel][0];  
  apc240set[8] = fre[apc240_channel][1];  
  apc240set[9] = fre[apc240_channel][2];  
	HAL_UART_Transmit(&hlpuart1, apc240set ,15, 0x01 );
	HAL_Delay(400);
	rtu.setFreMode = 0;
	Apc240ModeSet(Apc240Mode1);
	UartInit(38400);
}


