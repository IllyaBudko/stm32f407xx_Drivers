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

//PB9 -> SCL
//PB6 -> SDA

uint8_t some_data[] = "We are testing I2C master Tx\n";
I2C_Handle_t I2C1Handle;

void delay(void) {
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void I2C1_GPIO_Init(void) {
	GPIO_Handle_t I2CPin;
	I2CPin.pGPIO = GPIOB;
	I2CPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OD;
	I2CPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_PU;
	I2CPin.GPIO_PinConfig.GPIO_PinAltFuncMode = 4;
	I2CPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HS;

	//scl
	I2CPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPin);

	I2CPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GPIO_Init(&I2CPin);

}

void I2C1_Init(void) {
	I2C1Handle.pI2C = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}



void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIO = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HS;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIO = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OSPEED_HS;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PUPD_NONE;

	GPIO_Pclk_Ctrl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}

int main() {

	I2C1_GPIO_Init();

	I2C1_Init();

	GPIO_ButtonInit();

	I2C_PCtrl(I2C1, ENABLE);

	while(1) {
		while(! GPIO_Read_Pin(GPIOA, GPIO_PIN_0));
		delay();
	}

	I2C_Master_SendData(&I2C1Handle, some_data, strlen((char *)some_data), 0x68);

	while(1);






}










