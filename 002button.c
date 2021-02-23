/*
 * 002button.c
 *
 *  Created on: 20-Feb-2021
 *      Author: Training
 */

#include "stm32f407xx.h"
#define HIGH 1
#define BTN_PRESSED HIGH
void delay(void)
{
	for(uint32_t i=0; i< 4000000;i++)
	{

	}
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx=GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PD;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);
/*
 * button
 */
	GPIO_Handle_t GpioBtn;
	GpioLed.pGPIOx=GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IN;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
		GPIO_Init(&GpioBtn);



	while(1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ==BTN_PRESSED)
		{

		GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
		delay();
	    }
	}
	return 0;
}





