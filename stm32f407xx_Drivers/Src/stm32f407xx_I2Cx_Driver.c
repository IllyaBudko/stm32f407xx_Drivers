/*
 * stm32f407xx_I2Cx_Driver.c
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#include "stm32f407xx.h"
#include "stm32f407xx_I2Cx_Driver.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//I2Cx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//I2Cx Peripheral control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_PCtrl(I2Cx_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE) {
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else {
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

//I2Cx Peripheral clock setup
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_Pclk_Init(I2Cx_RegDef_t *pI2Cx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE) {
		if(pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

//I2Cx Initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_Init(I2Cx_Handle_t *pI2CHandle)
{

}

//I2Cx De-initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_DeInit(I2Cx_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1) {
		I2C1_RESET();
	}
	else if(pI2Cx == I2C2) {
		I2C2_RESET();
	}
	else if(pI2Cx == I2C3) {
		I2C3_RESET();
	}
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IRQ Configuration and handling APIs for I2Cx
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//IRQ Interrupt configuration APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
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

//IRQ Priority configuration APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shiftAmount;
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Peripheral control helper APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//I2Cx get flag status
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t I2C_GetFlagStatus(I2Cx_RegDef_t *pI2Cx, uint32_t flagName)
{
	if(pSPIx->SR & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//I2Cx application callback
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_AppEvCallback(I2Cx_Handle_t *pI2CHandle , uint8_t AppEv)
{

}







