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
void USART_PeriClockControl(USARTx_RegDef_t *pUSARTx, uint8_t ENorDI)
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
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX) {
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

	//word length selection
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	//Parity control and even odd selection
	if(pUSARTHandle->USART_Config.USART_ParityCtrl > USART_PARITY_DISABLE) {

		tempreg |= (1 << USART_CR1_PCE);

		if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_EN_ODD) {
			tempreg |= (1 << USART_CR1_PS);
		}
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;
	tempreg = 0;
	//CR2 configuration
	//stop bit selection
	pUSARTHandle->pUSARTx->CR2 |= pUSARTHandle->USART_Config.USART_NoOfStopBit << USART_CR2_STOP;

	//CR3 configuration
	if(pUSARTHandle->USART_Config.USART_HWFlowCtrl > 0) {
		uint8_t temp = pUSARTHandle->USART_Config.USART_HWFlowCtrl;
		switch(temp)
		{
		case 1:
			tempreg |= (1 << USART_CR3_CTSE);
			break;
		case 2:
			tempreg |= (1 << USART_CR3_RTSE);
			break;
		case 3:
			tempreg |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
			break;
		}
	}
	pUSARTHandle->pUSARTx->CR3 = tempreg;
	tempreg = 0;

	//baud rate setup
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BaudRate);

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
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			pData = (uint16_t *) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_DISABLE) {
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

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++) {
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			//We are going to receive 9bit data in a frame
			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_DISABLE) {
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else {
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		}
		else {
			//We are going to receive 8bit data in a frame
			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_DISABLE) {
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}
			else {
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX) {
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_TCIE);
	}
	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX) {
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}
	return rxstate;
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
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1 , temp2, temp3;

	uint16_t *pdata;
	/*************************Check for TC flag ********************************************/
	//Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);
	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);
	if(temp1 && temp2 ) {
		//this interrupt is because of TC
		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen ) {
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/
	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);
	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);
	if(temp1 && temp2 ) {
		//this interrupt is because of TXE
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX) {
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0) {
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_DISABLE) {
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=2;
					}
					else {
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen-=1;
					}
				}
				else {
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);
					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen-=1;
				}
			}
			if (pUSARTHandle->TxLen == 0 ) {
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 ) {
		//this interrupt is because of rxne
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX) {
			if(pUSARTHandle->RxLen > 0) {
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
					//We are going to receive 9bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_DISABLE) {
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen-=2;
					}
					else {
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen-=1;
					}
				}
				else {
					//We are going to receive 8bit data in a frame
					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityCtrl == USART_PARITY_DISABLE) {
						//No parity is used , so all 8bits will be of user data
						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}
					else {
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity
						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
					}
					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					 pUSARTHandle->RxLen-=1;
				}
			}//if of >0
			if(! pUSARTHandle->RxLen) {
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}
/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5
	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);
	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);
	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);
	if(temp1  && temp2 ) {
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);
		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}
/*************************Check for IDLE detection flag ********************************************/
	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);
	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);
	if(temp1 && temp2) {
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}
/*************************Check for Overrun detection flag ********************************************/
	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;
	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;
	if(temp1  && temp2 ) {
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}
/*************************Check for Error Flag ********************************************/
//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.
	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;
	if(temp2 ) {
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE)) {
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}
		if(temp1 & ( 1 << USART_SR_NF) ) {
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NF);
		}
		if(temp1 & ( 1 << USART_SR_ORE) ) {
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}

void USART_PeripheralControl(USARTx_RegDef_t *pUSARTx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE) {
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
	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6) {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2_Value();
  }
  else {
	   PCLKx = RCC_GetPCLK1_Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << 15)) {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 * BaudRate));
  }
  else {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (8 * BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8)) {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }
  else {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}











