#ifndef _apc240_h
#define _apc240_h
#include "stm32l0xx_hal.h"
typedef enum
{
	Apc240Mode1,
	Apc240Mode2,
	Apc240Mode3,
	Apc240Mode4	
} Apc240Mode;

typedef enum
{
   apc240_channel_0,
	 apc240_channel_1,
	 apc240_channel_2,
	 apc240_channel_3,
	 apc240_channel_4,
	 apc240_channel_5,
	 apc240_channel_6,
	 apc240_channel_7,
	 apc240_channel_8,
	 apc240_channel_9,
	 apc240_channel_10,
	 apc240_channel_11,
	 apc240_channel_12,
	 apc240_channel_13,
	 apc240_channel_14,
	 apc240_channel_15,
	 apc240_channel_16,
	 apc240_channel_17,
	 apc240_channel_18,
	 apc240_channel_19,
	 apc240_channel_20,
	 apc240_channel_21,
} Apc240_Channel;

typedef struct 
{
	uint8_t  telNumber[4][12];
	uint8_t  telNumberState[4];
} TelNumber;

typedef struct
{
	uint8_t rtuId;
	uint32_t id[3];
	uint8_t deviceFamily;
	uint8_t descripiton[30];
	Apc240_Channel  apc240_Channel;
	uint8_t chargeTime;
	uint8_t batChargeTime;
	uint8_t inputState[4];
	uint8_t coilStae[4];
	uint16_t voltage;
	uint16_t supportVoltage;
	uint16_t adcInput[4];
	uint16_t temperature;
	TelNumber telNumber;
	uint8_t setFreMode;
	uint8_t setMode;
	uint8_t lock;
	uint8_t voltageLowFlage;
	uint8_t awakeUp;
} RTU;
void Apc240SendWait(uint8_t * sendData, uint8_t sendLength);
void Apc240Init(void);
void Apc240ModeSet(Apc240Mode);
void Apc240Send(uint8_t *data,int lenght);
void Apc240SetFre(Apc240_Channel);
void UartInit(int);
#endif
