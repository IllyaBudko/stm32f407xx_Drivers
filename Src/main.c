/*
 * main.c
 *
 *  Created on: Jan 17, 2020
 *      Author: Illya Budko
 */

#include	<stdio.h>
#include	<stdint.h>
#include	<string.h>
#include	"stm32f407xx.h"

int main() {


	GPIOx_Handle_t userButton, led1, led2, led3, led4;

	userButton.GPIOx = GPIOA;
	userButton.GPIO_PinConfig.PinNumber = GPIO_PIN_0;
	userButton.GPIO_PinConfig.PinMode   = GPIO_MODE_IT_RT;
	userButton.GPIO_PinConfig.PinSpeed  = GPIO_OSPEED_HS;
	userButton.GPIO_PinConfig.PinPuPdControl = GPIO_PUPD_NONE;

	led1.GPIOx = GPIOD;
	led1.GPIO_PinConfig.PinNumber = GPIO_PIN_12;
	led1.GPIO_PinConfig.PinMode   = GPIO_MODE_OUT;
	led1.GPIO_PinConfig.PinOPType = GPIO_OPTYPE_PP;
	led1.GPIO_PinConfig.PinPuPdControl = GPIO_PUPD_NONE;

	led2.GPIOx = GPIOD;
	led2.GPIO_PinConfig.PinNumber = GPIO_PIN_13;
	led2.GPIO_PinConfig.PinMode   = GPIO_MODE_OUT;
	led2.GPIO_PinConfig.PinOPType = GPIO_OPTYPE_PP;
	led2.GPIO_PinConfig.PinPuPdControl = GPIO_PUPD_NONE;

	led3.GPIOx = GPIOD;
	led3.GPIO_PinConfig.PinNumber = GPIO_PIN_14;
	led3.GPIO_PinConfig.PinMode   = GPIO_MODE_OUT;
	led3.GPIO_PinConfig.PinOPType = GPIO_OPTYPE_PP;
	led3.GPIO_PinConfig.PinPuPdControl = GPIO_PUPD_NONE;

	led4.GPIOx = GPIOD;
	led4.GPIO_PinConfig.PinNumber = GPIO_PIN_15;
	led4.GPIO_PinConfig.PinMode   = GPIO_MODE_OUT;
	led4.GPIO_PinConfig.PinOPType = GPIO_OPTYPE_PP;
	led4.GPIO_PinConfig.PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Init(&userButton);
	GPIO_Init(&led1);
	GPIO_Init(&led2);
	GPIO_Init(&led3);
	GPIO_Init(&led4);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);

	while(1) {
		GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		for(uint32_t i=0; i<50000;i++);
		GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
		for(uint32_t i=0; i<50000;i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		for(uint32_t i=0; i<50000;i++);
		GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
		for(uint32_t i=0; i<50000;i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
		for(uint32_t i=0; i<50000;i++);
		GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
		for(uint32_t i=0; i<50000;i++);

		GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
		for(uint32_t i=0; i<50000;i++);
		GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
		for(uint32_t i=0; i<50000;i++);
	}
}
void EXTI0_IRQHandler (void) {

//	uint8_t volatile bitCheck = (((GPIOA->IDR) >> GPIO_PIN_0) & 1U);

	GPIO_IRQHandling(GPIO_PIN_0);
	while((((GPIOA->IDR) >> GPIO_PIN_0) & 1U) == SET) {
		GPIO_WritePin(GPIOD, GPIO_PIN_12, SET);
		GPIO_WritePin(GPIOD, GPIO_PIN_13, SET);
		GPIO_WritePin(GPIOD, GPIO_PIN_14, SET);
		GPIO_WritePin(GPIOD, GPIO_PIN_15, SET);
	}
	GPIO_WritePin(GPIOD, GPIO_PIN_12, RESET);
	GPIO_WritePin(GPIOD, GPIO_PIN_13, RESET);
	GPIO_WritePin(GPIOD, GPIO_PIN_14, RESET);
	GPIO_WritePin(GPIOD, GPIO_PIN_15, RESET);
}






