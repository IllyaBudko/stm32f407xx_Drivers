/*
 * stm32f407xx_USARTx_Driver.c
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#include "stm32f407xx.h"
#include "stm32f407xx_USARTx_Driver.h"

void USART_SetBaudRate(USARTx_RegDef_t *pUSARTx, uint32_t BaudRate);

//USARtx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART_PeriClockControl(USARTx_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(ENorDI == ENABLE) {
		if(pUSARTx == USART1) {
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2) {
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3) {
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4) {
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5) {
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
	}
	else{
		if(pUSARTx == USART1) {
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2) {
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3) {
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4) {
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5) {
			UART5_PCLK_DI();
		}
		else if(pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
	}
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	// CR1 configuration
	// mode selection
	if(pUSARTHandle->USART_Config->USART_Mode == USART_MODE_ONLY_RX) {
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config->USART_Mode == USART_MODE_ONLY_TX) {
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config->USART_Mode == USART_MODE_ONLY_TXRX) {
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	//word length selection
	tempreg |= pUSARTHandle->USART_Config->USART_WordLenght << USART_CR1_M;

	//Parity control and even odd selection
	if(pUSARTHandle->USART_Config->USART_ParityCtrl > USART_PARITY_DISABLE) {

		tempreg |= (1 << USART_CR1_PCE);

		if(pUSARTHandle->USART_Config->USART_ParityCtrl == USART_PARITY_EN_ODD) {
			tempreg |= (1 << USART_CR1_PS);
		}
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;
	tempreg = 0;
	//CR2 configuration
	//stop bit selection
	pUSARTHandle->pUSARTx->CR2 |= pUSARTHandle->USART_Config->USART_NoOfStopBit << USART_CR2_STOP;

	//CR3 configuration
	if(pUSARTHandle->USART_Config->USART_HWFlowCtrl > 0) {
		uint8_t temp = pUSARTHandle->USART_Config->USART_HWFlowCtrl;
		switch(temp)
		{
		case 1:
			tempreg |= (1 << USART_CR3_CTSE);
			break;
		}
		case 2:
			tempreg |= (1 << USART_CR3_RTSE);
			break;
		case 3:
			tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
			break;
	}
	pUSARTHandle->pUSARTx->CR3 = tempreg;
	tempreg = 0;

	//baud rate setup
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config->USART_BaudRate);

}

void USART_DeInit(USARTx_RegDef_t *pUSARTx)
{
			if(pUSARTx == USART1) {
				USART1_RESET();
			}
			else if(pUSARTx == USART2) {
				USART2_RESET();
			}
			else if(pUSARTx == USART3) {
				USART3_RESET();
			}
			else if(pUSARTx == UART4) {
				UART4_RESET();
			}
			else if(pUSARTx == UART5) {
				UART5_RESET();
			}
			else if(pUSARTx == USART6) {
				USART6_RESET();
			}
}

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pData;

	for( uint32_t i=0;i<Len;i++) {
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));
		if(pUSARTHandle->USART_Config->USART_WordLenght == USART_WORDLEN_9BITS) {
			pdata = (uint16_t *) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			if(pUSARTHandle->USART_Config->USART_ParityCtrl == USART_PARITY_DISABLE) {
				pTxBuffer++;
				pTxBuffer++;
			}
			else {
				pTxBuffer++;
			}
		}
		else {
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0xFF);
			pTxBuffer++;
		}
	}
	while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));
}

void USART_ReceiveData(USARTx_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len)
{

}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{

}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{

}

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shiftAmount;

}
void USART_IRQHandling(USART_Handle_t *pHandle)
{

}

void USART_PeripheralControl(USARTx_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnorDi == ENABLE) {
		pUSARTx->CR1 |= (1 << 13);
	}
	else {
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName) {
		return FLAG_SET;
	}
	else {
		return FLAG_RESET;
	}
}

void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint16_t FlagName)
{
	pUSARTx->SR &= ~(FlagName);
}

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv)
{

}

void USART_SetBaudRate(USARTx_RegDef_t *pUSARTx, uint32_t BaudRate)
{

}













