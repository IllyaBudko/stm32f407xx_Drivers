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

void delay(uint32_t n);
void blink(GPIOx_RegDef_t *pGPIOx, uint32_t pinNum, uint32_t timeDelay);

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

	uint32_t main_Time = 150000;

	while(1) {
		blink(GPIOD, GPIO_PIN_12, main_Time);
		blink(GPIOD, GPIO_PIN_13, main_Time);
		blink(GPIOD, GPIO_PIN_14, main_Time);
		blink(GPIOD, GPIO_PIN_15, main_Time);
	}
}
void EXTI0_IRQHandler (void) {

	GPIO_IRQHandling(GPIO_PIN_0);

	uint32_t pinCheck;

	pinCheck = (((GPIOD->ODR) >> GPIO_PIN_12) & 0xFF);

	uint32_t IT_Time = 100000;

	switch (pinCheck) {
	case 1:
		while((((GPIOA->IDR) >> GPIO_PIN_0) & 1U) == SET) {
			blink(GPIOD, GPIO_PIN_12, IT_Time);
			blink(GPIOD, GPIO_PIN_15, IT_Time);
			blink(GPIOD, GPIO_PIN_14, IT_Time);
			blink(GPIOD, GPIO_PIN_13, IT_Time);
		}
		break;
	case 2:
		while((((GPIOA->IDR) >> GPIO_PIN_0) & 1U) == SET) {
			blink(GPIOD, GPIO_PIN_13, IT_Time);
			blink(GPIOD, GPIO_PIN_12, IT_Time);
			blink(GPIOD, GPIO_PIN_15, IT_Time);
			blink(GPIOD, GPIO_PIN_14, IT_Time);
		}
		break;
	case 4:
		while((((GPIOA->IDR) >> GPIO_PIN_0) & 1U) == SET) {
			blink(GPIOD, GPIO_PIN_14, IT_Time);
			blink(GPIOD, GPIO_PIN_13, IT_Time);
			blink(GPIOD, GPIO_PIN_12, IT_Time);
			blink(GPIOD, GPIO_PIN_15, IT_Time);
		}
		break;
	case 8:
		while((((GPIOA->IDR) >> GPIO_PIN_0) & 1U) == SET) {
			blink(GPIOD, GPIO_PIN_15, IT_Time);
			blink(GPIOD, GPIO_PIN_14, IT_Time);
			blink(GPIOD, GPIO_PIN_13, IT_Time);
			blink(GPIOD, GPIO_PIN_12, IT_Time);
		}
		break;
	}
}

void delay(uint32_t n) {
	for(uint32_t i=0;i<n;i++);
}

void blink(GPIOx_RegDef_t *pGPIOx, uint32_t pinNum, uint32_t timeDelay) {
	GPIO_WritePin(pGPIOx, pinNum, SET);
	delay(timeDelay);
	GPIO_WritePin(pGPIOx, pinNum, RESET);
//	delay(timeDelay);
}












