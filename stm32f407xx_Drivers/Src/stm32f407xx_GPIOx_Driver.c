/*
 * stm32f407xx_GPIOx_Driver.c
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */
#include "stm32f407xx.h"
#include "stm32f407xx_GPIOx_Driver.h"

//GPIOx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GPIO_Pclk_Init(GPIOx_RegDef_t *pGPIOx, uint8_t ENorDI) {
	if(ENorDI == ENABLE) {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}
	else {
		if(pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}
}

void GPIO_Init(GPIOx_Handle_t *pGPIOHandle)
{
	GPIO_Pclk_Init(pGPIOHandle->GPIOx, ENABLE);
	if(pGPIOHandle->GPIO_PinConfig.PinMode < GPIO_MODE_ANALOG){
		(pGPIOHandle->GPIOx->MODER) &= ~(0x03 << 2 * (pGPIOHandle->GPIO_PinConfig.PinNumber));
		(pGPIOHandle->GPIOx->MODER) |=  (pGPIOHandle->GPIO_PinConfig.PinMode << 2 * (pGPIOHandle->GPIO_PinConfig.PinNumber));
	}
	else {
		if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT){
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RFT){
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		}
		uint8_t extiNumber = pGPIOHandle->GPIO_PinConfig.PinNumber / 4;
		uint8_t extiPinSelect = pGPIOHandle->GPIO_PinConfig.PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->GPIOx);

		switch(extiNumber) {
		case 0 :
			SYSCFG->EXTICR1 |= portCode << (extiPinSelect * 4);
			break;
		case 1 :
			SYSCFG->EXTICR2 |= portCode << (extiPinSelect * 4);
			break;
		case 2 :
			SYSCFG->EXTICR3 |= portCode << (extiPinSelect * 4);
			break;
		case 3 :
			SYSCFG->EXTICR4 |= portCode << (extiPinSelect * 4);
			break;
		}
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.PinNumber;
	}
	(pGPIOHandle->GPIOx->OSPEEDR) &= ~(0x03 << 2 * (pGPIOHandle->GPIO_PinConfig.PinNumber));
	(pGPIOHandle->GPIOx->OSPEEDR) |=  (pGPIOHandle->GPIO_PinConfig.PinSpeed << 2 * (pGPIOHandle->GPIO_PinConfig.PinNumber));

	(pGPIOHandle->GPIOx->PUPDR) &= ~(0x03 << 2 * (pGPIOHandle->GPIO_PinConfig.PinNumber));
	(pGPIOHandle->GPIOx->PUPDR) |=  (pGPIOHandle->GPIO_PinConfig.PinPuPdControl << 2 * (pGPIOHandle->GPIO_PinConfig.PinNumber));

	if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_OUT){
		(pGPIOHandle->GPIOx->OTYPER) &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.PinNumber);
		(pGPIOHandle->GPIOx->OTYPER) |=  (pGPIOHandle->GPIO_PinConfig.PinPuPdControl << pGPIOHandle->GPIO_PinConfig.PinNumber);
	}
	if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_AF){
		uint32_t AFRegSelect;
		uint8_t AFRegPinSelect;
		AFRegSelect = pGPIOHandle->GPIO_PinConfig.PinNumber / 8;
		AFRegPinSelect = pGPIOHandle->GPIO_PinConfig.PinNumber % 8;
		switch(AFRegSelect) {
		case 0:
			pGPIOHandle->GPIOx->AFRL &= ~(0xF << (4 * AFRegPinSelect));
			pGPIOHandle->GPIOx->AFRL |=  (pGPIOHandle->GPIO_PinConfig.PinAltFuncMode << (4 *AFRegPinSelect));
		case 1:
			pGPIOHandle->GPIOx->AFRH &= ~(0xF << (4 * AFRegPinSelect));
			pGPIOHandle->GPIOx->AFRH |=  (pGPIOHandle->GPIO_PinConfig.PinAltFuncMode << (4 * AFRegPinSelect));
		}
	}
}

void GPIO_Deinit(GPIOx_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA) {
		GPIOA_RESET();
	}
	else if(pGPIOx == GPIOB) {
		GPIOB_RESET();
	}
	else if(pGPIOx == GPIOC) {
		GPIOC_RESET();
	}
	else if(pGPIOx == GPIOD) {
		GPIOD_RESET();
	}
	else if(pGPIOx == GPIOE) {
		GPIOE_RESET();
	}
	else if(pGPIOx == GPIOE) {
		GPIOE_RESET();
	}
	else if(pGPIOx == GPIOF) {
		GPIOF_RESET();
	}
	else if(pGPIOx == GPIOG) {
		GPIOG_RESET();
	}
	else if(pGPIOx == GPIOH) {
		GPIOH_RESET();
	}
	else if(pGPIOx == GPIOI) {
		GPIOI_RESET();
	}
}

uint8_t GPIO_ReadPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t pinValue;
	pinValue = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return pinValue;
}

uint16_t GPIO_ReadPort(GPIOx_RegDef_t *pGPIOx){
	uint16_t pinValue = (uint16_t)pGPIOx->IDR;
	return pinValue;
}

void GPIO_WritePin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
//	(pGPIOx->ODR >> PinNumber) |= Value;
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WritePort(GPIOx_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR |= Value;
}

void GPIO_togglePin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

//IRQ Configuration and handling APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		if(IRQNumber < 31){
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber > 64 && IRQNumber < 96){
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else{
		if(IRQNumber < 31){
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		if(IRQNumber > 31 && IRQNumber < 64){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		if(IRQNumber > 64 && IRQNumber < 96){
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shiftAmount;
}

void GPIO_IRQHandling(uint8_t PinNumber){
	if(EXTI->PR & ( 1 << PinNumber)){
		EXTI->PR |= ( 1 << PinNumber);
	}
}

