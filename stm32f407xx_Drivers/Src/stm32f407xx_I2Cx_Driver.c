/*
 * stm32f407xx_I2Cx_Driver.c
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#include "stm32f407xx.h"
#include "stm32f407xx_I2Cx_Driver.h"



uint16_t AHB_PreScaler[8]  = {2,4,8,16,64,128,256,512};
uint8_t  APB1_PreScaler[4] = {2,4,8,16};

//Start condition helper function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void I2C_StartCondition(I2Cx_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

//Start condition helper function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void I2C_StopCondition(I2Cx_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

//address phase execution helper function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void I2C_ExecuteAddrPhase(I2Cx_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1);
	pI2Cx->DR = slaveAddr;
}

//address phase execution helper function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void I2C_ClearADDRFlag(I2Cx_RegDef_t *pI2Cx)
{
	uint32_t dummyRead;
	dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR1;
	(void)dummyRead;
}

uint32_t RCC_GetPLLOutputClk()
{
	return 0;
}

uint32_t RCC_GetPCLK1_Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = (RCC->CFGR >> 2) & (0x03);
	if(clksrc == 0) {
		SystemClk = 16000000;
	}
	else if(clksrc == 1) {
		SystemClk = 8000000;
	}
	else if(clksrc == 2) {
		SystemClk = RCC_GetPLLOutputClk();
	}

	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8) {
		ahbp = 1;
	}
	else {
		ahbp = AHB_PreScaler[temp];
	}

	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4) {
		apb1 = 1;
	}
	else {
		apb1 = APB1_PreScaler[temp-4];
	}

	pclk1 = (SystemClk / ahbp) / apb1;

	return pclk1;
}

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
	uint32_t tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKCtrl << 10;		//acknowledge control bit

	tempreg = 0;
	tempreg |= RCC_GetPCLK1_Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddr << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	uint16_t ccrValue = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		ccrValue = RCC_GetPCLK1_Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccrValue & 0xFFF);
	}
	else {
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_FMDUTY_2) {
			ccrValue = RCC_GetPCLK1_Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else {
			ccrValue = RCC_GetPCLK1_Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccrValue & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
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

//SPIx Get flag status
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t I2C_GetFlagStatus(I2Cx_RegDef_t *pI2Cx, uint32_t flagName)
{
	if(pI2Cx->SR1 & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//IRQ Interrupt configuration APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_MasterSendData (I2Cx_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr)
{
	I2C_StartCondition(pI2CHandle->pI2Cx);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
	I2C_ExecuteAddrPhase(pI2CHandle->pI2Cx, slaveAddr);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	while(Len > 0) {
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));
	I2C_StopCondition(pI2CHandle->pI2Cx);
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

//I2Cx application callback
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_AppEvCallback(I2Cx_Handle_t *pI2CHandle , uint8_t AppEv)
{

}







