/*
 * stm32f407xx_RCC_Driver.h
 *
 *  Created on: Feb 14, 2020
 *      Author: Illya Budko
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"



uint32_t RCC_GetPCLK1_Value(void);
uint32_t RCC_GetPCLK2_Value(void);
uint32_t RCC_GetPLLOutputClk(void);

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
