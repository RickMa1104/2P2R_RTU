#ifndef  _delay_h
#define  _delay_h
#include "stm32l0xx_hal.h"
#define  SYSCLK        2   
#define  A             6           //ѭ���ķ�cpu����
#define  B             3           // ��ʼ������
void delay_us(uint32_t) ;
void delay_ms(uint32_t) ;
void delay_s(uint32_t) ;
#endif
