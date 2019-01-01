#ifndef  _rtu_h
#define  _rtu_h
#include "stm32l0xx_hal.h"
#include <stdio.h>
#include "apc240.h"
#include "led.h"

#include "mbcrc.h"
#include "string.h"
#define  IdAddress                         ((uint32_t)0x08080000)  
#define  ChargeTimeAddress                 ((uint32_t)0x08080001)  
#define  BatTimeAddress                    ((uint32_t)0x08080002)  
#define  Apc240FreChannelAddress           ((uint32_t)0x08080003)  
#define  LockAddress                       ((uint32_t)0x08080004)
#define  LIU1V1                            ((uint32_t)0x08080005)  
#define  LIU1V2                            ((uint32_t)0x08080006)  
#define  LIU1V3                            ((uint32_t)0x08080007)  
#define  LIU1V4                            ((uint32_t)0x08080008)
#define  LIU2V1                            ((uint32_t)0x08080009)  
#define  LIU2V2                            ((uint32_t)0x0808000A)  
#define  LIU2V3                            ((uint32_t)0x0808000B)  
#define  LIU2V4                            ((uint32_t)0x0808000C)
#define  BACKMAXLEN                         200
#define  TEMP130_CAL_ADDR                  ((uint16_t*) ((uint32_t) 0x1FF8007E))
#define  TEMP30_CAL_ADDR                   ((uint16_t*) ((uint32_t) 0x1FF8007A))
#define  VREFINT_CAL											 ((uint16_t*) ((uint32_t) 0x1FF80078))
#define  VDD_CALIB                         ((uint16_t) (300))
#define  VDD_APPLI                         ((uint16_t) (330))
#endif
typedef enum
{
	close,open
} Operation;
void OperateCoil(Operation, int);
void RtuDateInit(void);
void WriteDateEeprom(uint32_t, uint8_t);
uint8_t ReadDateEeprom(uint32_t);
float ComputeTemperature(uint32_t);
void RtuSet(uint8_t * apc240RevDate, uint8_t apc240RecLen);
uint8_t UpdateInput(void);
void UpdateCoile(uint8_t);
void ClearRcvBuff(void);
void ClearRcvBuff(void);
void OutputVoltage(Operation operate, int coilNumber);
