#include "rtu.h"
#include "delay.h"
#include "apc240.h"
extern  RTU rtu;
extern  uint8_t    apc240RevDate[256];
extern  uint8_t           apc240RecLen;
extern  float vdda[4];
extern  uint8_t           coilFlag3 , coilFlag4 ;
//char backDate[BACKMAXLEN];
void WriteDateEeprom(uint32_t address, uint8_t date)
{
	 HAL_FLASHEx_DATAEEPROM_Unlock();
	 HAL_FLASHEx_DATAEEPROM_Program(TYPEPROGRAM_BYTE, address, date);
	 HAL_FLASHEx_DATAEEPROM_Lock();
}

uint8_t ReadDateEeprom(uint32_t address)
{
	return (*((uint8_t*)address));
}

void ClearRcvBuff()
{
		memset((void*)apc240RevDate, 0 , sizeof(uint8_t) * apc240RecLen);
		apc240RecLen = 0;
}

void RtuDateInit(void)
{
	rtu.rtuId = ReadDateEeprom(IdAddress) <= 247 ? ReadDateEeprom(IdAddress) : 0;
	rtu.chargeTime = ReadDateEeprom(ChargeTimeAddress) <= 15 ? ReadDateEeprom(ChargeTimeAddress) : 0;
	rtu.batChargeTime = ReadDateEeprom(BatTimeAddress) <= 200 ?  ReadDateEeprom(BatTimeAddress) : 0;
	rtu.lock = ReadDateEeprom(LockAddress);
	rtu.apc240_Channel = (Apc240_Channel)ReadDateEeprom(Apc240FreChannelAddress) <= 21 ? 
	(Apc240_Channel)ReadDateEeprom(Apc240FreChannelAddress):apc240_channel_0;
	rtu.deviceFamily = 1;
	rtu.id[0] =  *(__IO uint32_t*)(0x1FF80050);
	rtu.id[1] =  *(__IO uint32_t*)(0x1FF80050 + 4);
	rtu.id[2] =  *(__IO uint32_t*)(0x1FF80050 + 0x14);
	snprintf((char *)rtu.descripiton , 30, "Normal Rtu");
	rtu.awakeUp = 0;
}

void OutputVoltage(Operation operate, int coilNumber)
{
		if(operate==open)
      {
				HAL_GPIO_WritePin(FMCZ_GPIO_Port, FMCZ_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(FMCF_GPIO_Port, FMCF_Pin, GPIO_PIN_RESET);

      
        switch (coilNumber)
            {
                    case 1 :
											HAL_GPIO_WritePin(FM1Z_GPIO_Port, FM1Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM1F_GPIO_Port, FM1F_Pin, GPIO_PIN_SET);
											break;	
                    case 2 :
											HAL_GPIO_WritePin(FM2Z_GPIO_Port, FM2Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM2F_GPIO_Port, FM2F_Pin, GPIO_PIN_SET);
											break;	
                    case 3 :
											HAL_GPIO_WritePin(FM3Z_GPIO_Port, FM3Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM3F_GPIO_Port, FM3F_Pin, GPIO_PIN_SET);
											break;	
                    case 4 :
											HAL_GPIO_WritePin(FM4Z_GPIO_Port, FM4Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM4F_GPIO_Port, FM4F_Pin, GPIO_PIN_SET);
											break;	
            }
			
				HAL_Delay(3000);
				    
    }
    else if(operate==close)
    {

				HAL_GPIO_WritePin(FMCZ_GPIO_Port, FMCZ_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(FMCF_GPIO_Port, FMCF_Pin, GPIO_PIN_RESET);
      switch (coilNumber)
            {
                        case 1 :
											HAL_GPIO_WritePin(FM1Z_GPIO_Port, FM1Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM1F_GPIO_Port, FM1F_Pin, GPIO_PIN_RESET);
											break;	
                    case 2 :
											HAL_GPIO_WritePin(FM2Z_GPIO_Port, FM2Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM2F_GPIO_Port, FM2F_Pin, GPIO_PIN_RESET);
											break;	
                    case 3 :
											HAL_GPIO_WritePin(FM3Z_GPIO_Port, FM3Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM3F_GPIO_Port, FM3F_Pin, GPIO_PIN_RESET);
											break;	
                    case 4 :
											HAL_GPIO_WritePin(FM4Z_GPIO_Port, FM4Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM4F_GPIO_Port, FM4F_Pin, GPIO_PIN_RESET);
											break;	
            }
    }
}

void OperateCoil(Operation operate, int coilNumber)
{
	OutputVoltage(close, 1);
	OutputVoltage(close, 2);
	HAL_Delay(100);
	if(operate==open)
      {
				HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_RESET);
				if(rtu.chargeTime != 0)
				{
					for(uint32_t i = 0; i <10000 * rtu.chargeTime; i++)
					{
						HAL_GPIO_WritePin(PWM_OUT_GPIO_Port, PWM_OUT_Pin, GPIO_PIN_SET);		
						HAL_GPIO_WritePin(PWM_OUT_GPIO_Port, PWM_OUT_Pin, GPIO_PIN_RESET);
					}
				}
				
				HAL_GPIO_WritePin(FMCZ_GPIO_Port, FMCZ_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(FMCF_GPIO_Port, FMCF_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_SET);
      
        switch (coilNumber)
            {
                    case 1 :
											HAL_GPIO_WritePin(FM1Z_GPIO_Port, FM1Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM1F_GPIO_Port, FM1F_Pin, GPIO_PIN_SET);
											break;	
                    case 2 :
											HAL_GPIO_WritePin(FM2Z_GPIO_Port, FM2Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM2F_GPIO_Port, FM2F_Pin, GPIO_PIN_SET);
											break;	
                    case 3 :
											HAL_GPIO_WritePin(FM3Z_GPIO_Port, FM3Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM3F_GPIO_Port, FM3F_Pin, GPIO_PIN_SET);
						   				coilFlag3 = 1;
 											break;	
                    case 4 :
											HAL_GPIO_WritePin(FM4Z_GPIO_Port, FM4Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM4F_GPIO_Port, FM4F_Pin, GPIO_PIN_SET);
											coilFlag4 = 1;
											break;	
            }
				if(rtu.chargeTime != 0)		
				{
					HAL_Delay(100);
				}
//				HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_SET);
				if(rtu.batChargeTime != 0)
				{
					HAL_Delay(rtu.batChargeTime * 100);
				}
//				HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(FMCZ_GPIO_Port, FMCZ_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(FMCF_GPIO_Port, FMCF_Pin, GPIO_PIN_RESET);
        
        switch (coilNumber)
            {
                    case 1 :
											HAL_GPIO_WritePin(FM1Z_GPIO_Port, FM1Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM1F_GPIO_Port, FM1F_Pin, GPIO_PIN_RESET);
											break;	
                    case 2 :
											HAL_GPIO_WritePin(FM2Z_GPIO_Port, FM2Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM2F_GPIO_Port, FM2F_Pin, GPIO_PIN_RESET);
											break;	
                    case 3 :
											HAL_GPIO_WritePin(FM3Z_GPIO_Port, FM3Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM3F_GPIO_Port, FM3F_Pin, GPIO_PIN_RESET);
											break;	
                    case 4 :
											HAL_GPIO_WritePin(FM4Z_GPIO_Port, FM4Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM4F_GPIO_Port, FM4F_Pin, GPIO_PIN_RESET);
											break;	
            }      
    }
    else if(operate==close)
    {
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_RESET);
			if(rtu.chargeTime != 0)	
			{
				for(uint32_t i = 0; i <10000 * rtu.chargeTime; i++)
				{
					HAL_GPIO_WritePin(PWM_OUT_GPIO_Port, PWM_OUT_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(PWM_OUT_GPIO_Port, PWM_OUT_Pin, GPIO_PIN_RESET);
				}
			}
		
			HAL_GPIO_WritePin(FMCZ_GPIO_Port, FMCZ_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FMCF_GPIO_Port, FMCF_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_SET);
      switch (coilNumber)
            {
                     case 1 :
											HAL_GPIO_WritePin(FM1Z_GPIO_Port, FM1Z_Pin, GPIO_PIN_SET);
										  HAL_GPIO_WritePin(FM1F_GPIO_Port, FM1F_Pin, GPIO_PIN_RESET);
											break;	
                    case 2 :
											HAL_GPIO_WritePin(FM2Z_GPIO_Port, FM2Z_Pin, GPIO_PIN_SET);
										  HAL_GPIO_WritePin(FM2F_GPIO_Port, FM2F_Pin, GPIO_PIN_RESET);
											break;	
                    case 3 :
											HAL_GPIO_WritePin(FM3Z_GPIO_Port, FM3Z_Pin, GPIO_PIN_SET);
										  HAL_GPIO_WritePin(FM3F_GPIO_Port, FM3F_Pin, GPIO_PIN_RESET);
											coilFlag3 = 0;
											break;	
                    case 4 :
											HAL_GPIO_WritePin(FM4Z_GPIO_Port, FM4Z_Pin, GPIO_PIN_SET);
										  HAL_GPIO_WritePin(FM4F_GPIO_Port, FM4F_Pin, GPIO_PIN_RESET);
											coilFlag4 = 0;
											break;	
            }
			if(rtu.chargeTime != 0)		
			{				
				HAL_Delay(100);
			}
	//		HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_SET);
			if(rtu.batChargeTime != 0)
			{
				HAL_Delay(rtu.batChargeTime * 100);
			}
//			HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FMCZ_GPIO_Port, FMCZ_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(FMCF_GPIO_Port, FMCF_Pin, GPIO_PIN_RESET);			
   
      switch (coilNumber)
            {
                    case 1 :
											HAL_GPIO_WritePin(FM1Z_GPIO_Port, FM1Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM1F_GPIO_Port, FM1F_Pin, GPIO_PIN_RESET);
											break;	
                    case 2 :
											HAL_GPIO_WritePin(FM2Z_GPIO_Port, FM2Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM2F_GPIO_Port, FM2F_Pin, GPIO_PIN_RESET);
											break;	
                    case 3 :
											HAL_GPIO_WritePin(FM3Z_GPIO_Port, FM3Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM3F_GPIO_Port, FM3F_Pin, GPIO_PIN_RESET);
											break;	
                    case 4 :
											HAL_GPIO_WritePin(FM4Z_GPIO_Port, FM4Z_Pin, GPIO_PIN_RESET);
										  HAL_GPIO_WritePin(FM4F_GPIO_Port, FM4F_Pin, GPIO_PIN_RESET);
											break;	
            }
    }
}

void RtuSet(uint8_t * apc240RevDate, uint8_t apc240RecLen)
{
	if(usMBCRC16( apc240RevDate, apc240RecLen) == 0)
	{
		if(apc240RevDate[0] == 0xff &&  apc240RevDate[1] == 0xff && apc240RevDate[2] > 0 && apc240RevDate[2]<= 247 && apc240RevDate[3] <= 15
		&& apc240RevDate[4] <= 200 && (apc240RevDate[3] + apc240RevDate[4]) > 0 && apc240RevDate[5] <= 21	&& apc240RecLen ==8)
		{
			rtu.rtuId = apc240RevDate[2];
			rtu.chargeTime = apc240RevDate[3];
			rtu.batChargeTime = apc240RevDate[4];
			rtu.apc240_Channel = (Apc240_Channel)apc240RevDate[5];
			WriteDateEeprom(IdAddress, rtu.rtuId);
			WriteDateEeprom(ChargeTimeAddress, rtu.chargeTime);
			WriteDateEeprom(BatTimeAddress, rtu.batChargeTime);
			WriteDateEeprom(Apc240FreChannelAddress, rtu.apc240_Channel);
			Apc240SendWait((uint8_t *)apc240RevDate, apc240RecLen);
			ClearRcvBuff();
			HAL_Delay(3000);
			HAL_NVIC_SystemReset();
		}		
	}
}

float ComputeTemperature(uint32_t measure)
{
	float  temperature;
	temperature = ((measure * vdda[3]*100/ 300.0)- (int32_t) *TEMP30_CAL_ADDR );
	temperature = temperature * (int32_t)(130 - 30);
	temperature = temperature / (int32_t)(*TEMP130_CAL_ADDR -*TEMP30_CAL_ADDR);
	temperature = temperature + 30;
	return(temperature);
}

uint8_t UpdateInput()
{
	uint8_t temp = 0;
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
	{
		rtu.inputState[0] = 1;
		temp |= 1;
	}
	else
	{
		rtu.inputState[0] = 0;
		temp &= 0xfe;
	}
//	
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
//	{
//		rtu.inputState[1] = 1;
//		temp |= 2;
//	}
//	else
//	{
//		rtu.inputState[1] = 0;
//		temp &= 0xfd;
//	}
//	
//	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
//	{
//		rtu.inputState[2] = 1;
//		temp |= 4;
//	}
//	else
//	{
//		rtu.inputState[2] = 0;
//		temp &= 0xfb;
//	}
	
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET)
	{
		rtu.inputState[3] = 1;
		temp |= 8;
	}
	else
	{
		rtu.inputState[3] = 0;
		temp &= 0xf7;
	}
	return temp;
}
