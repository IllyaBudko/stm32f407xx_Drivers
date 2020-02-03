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

static void I2C_StartCondition(I2Cx_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_StopCondition(I2Cx_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddrPhaseWrite(I2Cx_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr &= ~(1);
	pI2Cx->DR = slaveAddr;
}

static void I2C_ExecuteAddrPhaseRead(I2Cx_RegDef_t *pI2Cx, uint8_t slaveAddr)
{
	slaveAddr = slaveAddr << 1;
	slaveAddr |= 1;
	pI2Cx->DR = slaveAddr;
}

static void I2C_ClearADDRFlag(I2Cx_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
		// master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			if(pI2CHandle->RxSize == 1) {
				//disable ack
				I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
				//clear addr flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else{
			//clear addr flag
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}
	else {
		//slave mode
		//clear addr flag
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

void I2C_CloseReceiveData (I2Cx_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	I2C_ManageAck(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_CloseSendData (I2Cx_Handle_t *pI2CHandle) {
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
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

//I2Cx APIs
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

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		tempreg = (RCC_GetPCLK1_Value() / 1000000U) + 1;
	}
	else
	{
		tempreg = ((RCC_GetPCLK1_Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;
}

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

uint8_t I2C_GetFlagStatus(I2Cx_RegDef_t *pI2Cx, uint32_t flagName)
{
	if(pI2Cx->SR1 & flagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_MasterSendData (I2Cx_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr)
{
	I2C_StartCondition(pI2CHandle->pI2Cx);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
	I2C_ExecuteAddrPhaseWrite(pI2CHandle->pI2Cx, slaveAddr);

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));
	I2C_ClearADDRFlag(pI2CHandle);

	while(Len > 0) {
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));
	if(Sr == I2C_DISABLE_SR)
		I2C_StopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData (I2Cx_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr)
{
	I2C_StartCondition(pI2CHandle->pI2Cx);
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	I2C_ExecuteAddrPhaseRead(pI2CHandle->pI2Cx, slaveAddr);
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	I2C_ClearADDRFlag(pI2CHandle);

	if(Len == 1) {
		I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
		I2C_StopCondition(pI2CHandle->pI2Cx);
		I2C_ClearADDRFlag(pI2CHandle);
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
		if(Sr == I2C_DISABLE_SR)
			I2C_StopCondition(pI2CHandle->pI2Cx);
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(Len > 1) {
		I2C_ClearADDRFlag(pI2CHandle);
		for(uint32_t i=0;i<Len;i--) {
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));
			if(Len == 2) {
				I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
				if(Sr == I2C_DISABLE_SR)
					I2C_StopCondition(pI2CHandle->pI2Cx);
			}
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	if(pI2CHandle->I2C_Config.I2C_ACKCtrl == I2C_ACK_EN) {
		I2C_ManageAck(pI2CHandle->pI2Cx, ENABLE);
	}
}

uint8_t I2C_Master_IT_SendData(I2Cx_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
		if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) {
			pI2CHandle->pTxBuffer = pTxBuffer;
			pI2CHandle->TxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->devAddr = slaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_StartCondition(pI2CHandle->pI2Cx);
			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		}
	return busystate;
}

uint8_t I2C_Master_IT_ReceiveData(I2Cx_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
		if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
		{
			pI2CHandle->pRxBuffer = pRxBuffer;
			pI2CHandle->RxLen = Len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
			pI2CHandle->devAddr = slaveAddr;
			pI2CHandle->Sr = Sr;

			//Implement code to Generate START Condition
			I2C_StartCondition(pI2CHandle->pI2Cx);
			//Implement the code to enable ITBUFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
			//Implement the code to enable ITEVFEN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
			//Implement the code to enable ITERREN Control Bit
			pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
		}
	return busystate;
}

//IRQ Configuration and handling APIs for I2Cx
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprxSection = IRQNumber % 4;
	uint8_t shiftAmount = (8 * iprxSection) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shiftAmount;
}

static void I2C_MasterHandleTXE_IT(I2Cx_Handle_t *pI2CHandle) {
	if(pI2CHandle->TxLen > 0) {
		//load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		//decrement TxLen
		pI2CHandle->TxLen--;
		//Increment buffer address
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNE_IT(I2Cx_Handle_t *pI2CHandle) {
//data reception
	if(pI2CHandle->RxSize == 1) {
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;

	}
	if(pI2CHandle->RxSize > 1) {
		if(pI2CHandle->RxLen == 2) {
			I2C_ManageAck(pI2CHandle->pI2Cx, DISABLE);
		}
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0) {
		I2C_StopCondition(pI2CHandle->pI2Cx);
		I2C_CloseReceiveData(pI2CHandle);
		I2C_AppEvCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_EV_IRQHandling(I2Cx_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3) {
		//SB is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			I2C_ExecuteAddrPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->devAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			I2C_ExecuteAddrPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->devAddr);
		}
	}
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3) {
		//ADDR is set
		I2C_ClearADDRFlag(pI2CHandle);
	}
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3) {
		//BTF is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)) {
				//BTF, TXE =1
				if(pI2CHandle->TxLen == 0) {
					//generate STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR) {
						I2C_StopCondition(pI2CHandle->pI2Cx);
					}
					//reset all member elements of handle structure
					I2C_CloseSendData(pI2CHandle);
					//notify application event complete
					I2C_AppEvCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			;
		}
	}
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3) {
		//STOPF is set
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		I2C_AppEvCallback(pI2CHandle, I2C_EV_STOP);
	}
	//5. Handle For interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3) {
		//check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			//TXE is set
			// data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				I2C_MasterHandleTXE_IT(pI2CHandle);
			}
		}
	}
	//6. Handle For interrupt generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3) {
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) {
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
				I2C_MasterHandleRXNE_IT(pI2CHandle);
			}
		}
	}
}

void I2C_ERR_IRQHandling(I2Cx_Handle_t *pI2CHandle)
{

}

//Peripheral control helper APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_AppEvCallback(I2Cx_Handle_t *pI2CHandle , uint8_t AppEv)
{

}

void I2C_ManageAck(I2Cx_RegDef_t *pI2Cx, uint8_t ENorDi)
{
	if(ENorDi == ENABLE) {
		pI2Cx->CR1 |= (1 << 10);
	}
	else {
		pI2Cx->CR1 &= ~(1 << 10);
	}
}














