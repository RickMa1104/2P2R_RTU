#include"led.h"
void LedOn()
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void LedOff()
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}
void LedToggle()
{
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
