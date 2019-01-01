#include "delay.h"
void wait(unsigned long n)
 {
		do{
         n--;
      }while(n);
 }

void delay_us(uint32_t nus)  
{
	wait(((nus)*(SYSCLK)-(B))/(A));
}	

void delay_ms(uint32_t nms) 
{
	 delay_us((nms)*1000);
} 
void delay_s(uint32_t ns) 
{
		delay_ms((ns)*1000);
}
