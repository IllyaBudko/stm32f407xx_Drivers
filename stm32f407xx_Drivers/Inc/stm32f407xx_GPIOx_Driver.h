/*
 * stm32f407xx_GPIOx_Driver.h
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#ifndef INC_STM32F407XX_GPIOX_DRIVER_H_
#define INC_STM32F407XX_GPIOX_DRIVER_H_

#include "stm32f407xx.h"



//GPIOx register definition structures
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//GPIO pin configuration definition structure
typedef struct
{
	uint8_t PinNumber;					//Pin number select
	uint8_t PinMode;					//Pin mode select
	uint8_t PinSpeed;					//Pin speed select
	uint8_t PinPuPdControl;				//Pin pull up pull down select
	uint8_t PinOPType;					//Pin output type select
	uint8_t PinAltFuncMode;				//Pin alternate function select
}GPIOx_PinConfig_t;

//GPIO handle definition structure
typedef struct
{
	GPIOx_RegDef_t *GPIOx;				//Port select
	GPIOx_PinConfig_t GPIO_PinConfig;	//Pin configuration select for specific port
}GPIOx_Handle_t;



//GPIOx Pin number definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

//GPIOx Pin Configuration definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//GPIO PinConfig modes definitions
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

//GPIO PinConfig speed definitions
#define GPIO_OSPEED_LS		0
#define GPIO_OSPEED_MS		1
#define GPIO_OSPEED_HS		2
#define GPIO_OSPEED_VHS		3

//GPIO PinConfig pull up pull down control definitions
#define GPIO_PUPD_NONE		0
#define GPIO_PUPD_PU		1
#define GPIO_PUPD_PD		2

//GPIO PinConfig output type definitions
#define GPIO_OPTYPE_PP		0
#define GPIO_OPTYPE_OD		1

//GPIO PinConfig alternate function mode definitions
#define GPIO_ALTFUNC_0		0
#define GPIO_ALTFUNC_1		1
#define GPIO_ALTFUNC_2		2
#define GPIO_ALTFUNC_3		3
#define GPIO_ALTFUNC_4		4
#define GPIO_ALTFUNC_5		5
#define GPIO_ALTFUNC_6		6
#define GPIO_ALTFUNC_7		7
#define GPIO_ALTFUNC_8		8
#define GPIO_ALTFUNC_9		9
#define GPIO_ALTFUNC_10		10
#define GPIO_ALTFUNC_11		11
#define GPIO_ALTFUNC_12		12
#define GPIO_ALTFUNC_13		13
#define GPIO_ALTFUNC_14		14
#define GPIO_ALTFUNC_15		15

//GPIOx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GPIO_Pclk_Init(GPIOx_RegDef_t *pGPIOx, uint8_t ENorDI);
void GPIO_Init(GPIOx_Handle_t *pGPIOHandle);
void GPIO_Deinit(GPIOx_RegDef_t *pGPIOx);

uint8_t GPIO_ReadPin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadPort(GPIOx_RegDef_t *pGPIOx);

void GPIO_WritePin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WritePort(GPIOx_RegDef_t *pGPIOx, uint16_t Value);

void GPIO_togglePin(GPIOx_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration and handling APIs for GPIOx
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIOX_DRIVER_H_ */
