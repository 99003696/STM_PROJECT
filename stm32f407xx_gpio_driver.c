#include "stm32f407xx_gpio_driver.h"

/*
 * to control clock setting
 */
void GPIO_PeriClockControl(Gpio_RegDef_t *pGPIOX,uint8_t EnorDi)
{
	if(pGPIOX == GPIOA)
	{
		GPIOA_PCLK_EN();
	}else if(pGPIOX == GPIOB)
	{
		GPIOB_PCLK_EN();
	}else if(pGPIOX == GPIOC)
	{
		GPIOC_PCLK_EN();
	}else if(pGPIOX == GPIOD)
	{
		GPIOD_PCLK_EN();
	}else if(pGPIOX == GPIOE)
	{
		GPIOE_PCLK_EN();
	}
	else{
		if(pGPIOX != GPIOA)
			{
				GPIOA_PCLK_DIS();
			}else if(pGPIOX != GPIOB)
			{
				GPIOB_PCLK_DIS();
			}else if(pGPIOX != GPIOC)
			{
				GPIOC_PCLK_DIS();
			}else if(pGPIOX != GPIOD)
			{
				GPIOD_PCLK_DIS();
			}else if(pGPIOX != GPIOE)
			{
				GPIOE_PCLK_DIS();
			}
		}

}

/*
 *  To initialize and deinitialize the GPIO
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	//Configure the mode of GPIOX
	uint32_t temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear the value
	pGPIOHandle->pGPIOx->MODER |=temp; //set the value

	// Configure the speed

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |=temp;

	// GPIO port up/pull down register

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<< (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |=temp;

	// GPIO Output Type register

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;

	// GPIO alternate function low register

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
	uint8_t temp1, temp2;
	temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
	temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
	pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xf<<(4* temp2));
	pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (4* temp2));
    }
}
void GPIO_DeInit(Gpio_RegDef_t *pGPIOX){
	if(pGPIOX == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOX == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOX == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOX == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOX == GPIOE)
	{
		GPIOE_REG_RESET();
	}

}

/*
 * to Read Input from GPIO
 */

uint8_t GPIO_ReadFromInputPin(Gpio_RegDef_t *pGPIOX, uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOX->IDR >> PinNumber)& 0x00000001);
	return value;

}
uint16_t GPIO_ReadFromInputPort(Gpio_RegDef_t *pGPIOX){
	uint16_t value;
	value=pGPIOX->IDR;
	return value;

}

/*
 * to Write output to GPIO
 */

void GPIO_WriteToOutputPin(Gpio_RegDef_t *pGPIOX, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET)
	{
	   pGPIOX -> ODR |= (1<<PinNumber);
	}else{
		 pGPIOX -> ODR &= ~(1<<PinNumber);
	}

 }
void GPIO_WriteToOutputPort(Gpio_RegDef_t *pGPIOX,uint16_t value){

	  pGPIOX -> ODR =value;
}

/*
 * to Toggle the output in GPIO
 */

void GPIO_ToggleOutputPin(Gpio_RegDef_t *pGPIOX,uint8_t PinNumber){

	pGPIOX -> ODR ^= (1<<PinNumber);
}
