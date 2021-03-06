/*
 * stm32f407xx_SPIx_Driver.c
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#include "stm32f407xx.h"
#include "stm32f407xx_SPIx_Driver.h"

//SPIx Helper functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SPI_PCtrl(SPIx_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

uint8_t SPI_GetFlagStatus(SPIx_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

static void SPI_TXE_ITHandle(SPIx_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		pSPIHandle->pSPIx->DR = *((int16_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else {
		pSPIHandle->pSPIx->DR = *((int8_t *)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(! pSPIHandle->TxLen) {
		SPI_CloseTransmission(pSPIHandle);
		SPI_AppEvCallback(pSPIHandle, SPI_EVENT_CMPLT_TX);				//to be implemented by application
	}
}

static void SPI_RXNE_ITHandle(SPIx_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		pSPIHandle->pSPIx->DR = *((int16_t *)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;
	}
	else {
		pSPIHandle->pSPIx->DR = *((int8_t *)pSPIHandle->pRxBuffer);
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(! pSPIHandle->RxLen) {
		SPI_CloseReception(pSPIHandle);
		SPI_AppEvCallback(pSPIHandle, SPI_EVENT_CMPLT_RX);				//to be implemented by application
	}
}

static void SPI_OVRERR_ITHandle(SPIx_Handle_t *pSPIHandle)
{
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_RX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_AppEvCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_Clear_OVRFlag(SPIx_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

void SPI_CloseTransmission(SPIx_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPIx_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_AppEvCallback(SPIx_Handle_t *pSPIHandle, uint8_t AppEv){

}

void  SPI_SSIConfig(SPIx_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE) {
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	}
	else {
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

void  SPI_SSOEConfig(SPIx_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE) {
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	}
	else {
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}

//SPIx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SPI_Pclk_Init(SPIx_RegDef_t *pSPIx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE) {
		if(pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if(pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	}
	else{
		if(pSPIx == SPI1) {
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}

void SPI_Init(SPIx_Handle_t *pSPIHandle)
{
	SPI_Pclk_Init(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode) << SPI_CR1_MSTR;
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
		tempreg |=  (1 << SPI_CR1_BIDIMODE) ;
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SX_RX) {
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |=  (1 << SPI_CR1_RXONLY);
	}

	tempreg |= (pSPIHandle->SPIConfig.SPI_SClkSpeed) << SPI_CR1_BR;
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF) << SPI_CR1_DFF;
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL) << SPI_CR1_CPOL;
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA) << SPI_CR1_CPHA;

	(pSPIHandle->pSPIx->CR1) = tempreg;
}

void SPI_DeInit(SPIx_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1) {
		SPI1_RESET();
	}
	else if(pSPIx == SPI2) {
		SPI2_RESET();
	}
	else if(pSPIx == SPI3) {
		SPI3_RESET();
	}
}

void SPI_SendData(SPIx_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len != 0) {
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			pSPIx->DR = *((int16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else {
			pSPIx->DR = *((int8_t *)pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData (SPIx_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len != 0) {
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_RXNE) == FLAG_SET);
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			*((int16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else {
			*((int8_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_IT_SendData(SPIx_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_TX) {
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_BUSY_TX;
		pSPIHandle->pSPIx->CR2 = (1 << SPI_CR2_TXEIE);
	}
	// data transmission to be handled by isr code
	return state;
}

uint8_t SPI_IT_ReceiveData (SPIx_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_RX) {
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;
			pSPIHandle->RxState = SPI_BUSY_TX;
			pSPIHandle->pSPIx->CR2 = (1 << SPI_CR2_RXNEIE);
		}
		//data reception to be handled by isr code
		return state;
}

//IRQ Configuration and handling APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shiftAmount;
}

void SPI_IRQHandling(SPIx_Handle_t *pSPIHandle)
{
	uint8_t temp1;
	uint8_t temp2;

	temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2) {
		SPI_TXE_ITHandle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2) {
		SPI_RXNE_ITHandle(pSPIHandle);
	}

	temp1 = pSPIHandle->pSPIx->SR  & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2) {
		SPI_OVRERR_ITHandle(pSPIHandle);
	}
}












