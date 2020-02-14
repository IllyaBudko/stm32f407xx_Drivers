/*
 * stm32f407xx_USARTx_Driver.h
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#ifndef INC_STM32F407XX_USARTX_DRIVER_H_
#define INC_STM32F407XX_USARTX_DRIVER_H_

#include "stm32f407xx.h"

typedef volatile struct
{
	uint8_t  USART_Mode;  //
	uint32_t USART_BaudRate;
	uint8_t  USART_NoOfStopBit;
	uint8_t  USART_WordLength; //
	uint8_t  USART_ParityCtrl; //
	uint8_t  USART_HWFlowCtrl;
}USART_Config_t;

typedef volatile struct
{
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
	USARTx_RegDef_t *pUSARTx;
	USART_Config_t   USART_Config;
}USART_Handle_t;

//USARTx Configuration definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// USART Word length
#define USART_MODE_ONLY_TX 				0
#define USART_MODE_ONLY_RX 				1
#define USART_MODE_TXRX 				2

// USART Baud Rate
#define USART_STD_BAUD_1200				1200
#define USART_STD_BAUD_2400				2400
#define USART_STD_BAUD_4800				4800
#define USART_STD_BAUD_9600				9600
#define USART_STD_BAUD_14400			14400
#define USART_STD_BAUD_19200			19200
#define USART_STD_BAUD_38400			38400
#define USART_STD_BAUD_57600			57600
#define USART_STD_BAUD_115200 			115200
#define USART_STD_BAUD_230400 			230400
#define USART_STD_BAUD_460800 			460800
#define USART_STD_BAUD_921600 			921600
#define USART_STD_BAUD_2M 				2000000
#define SUART_STD_BAUD_3M 				3000000

// USART Parity control
#define USART_PARITY_EN_ODD				2
#define USART_PARITY_EN_EVEN			1
#define USART_PARITY_DISABLE			0

// USART Word length
#define USART_WORDLEN_8BITS				0
#define USART_WORDLEN_9BITS				1

// USART stop bit length
#define USART_STOPBITS_1				0
#define USART_STOPBITS_0_5				1
#define USART_STOPBITS_2				2
#define USART_STOPBITS_1_5				3

// USART Hardware flow control
#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_CTS			1
#define USART_HW_FLOW_CTRL_RTS			2
#define USART_HW_FLOW_CTRL_CTS_RTS		3

//USARTx Status flags
#define USART_FLAG_PE					0
#define USART_FLAG_FE					1
#define USART_FLAG_NF					2
#define USART_FLAG_ORE					3
#define USART_FLAG_IDLE					4
#define USART_FLAG_RXNE					5
#define USART_FLAG_TC					6
#define USART_FLAG_TXE					7
#define USART_FLAG_LBD					8
#define USART_FLAG_CTS					9

//USARTx busy
#define USART_BUSY_IN_RX				1
#define USART_BUSY_IN_TX				2
#define USART_READY						0

//USART event and error flags
#define USART_EVENT_TX_CMPLT			0
#define USART_EVENT_RX_CMPLT			1
#define USART_EVENT_IDLE				2
#define USART_EVENT_CTS					3
#define USART_EVENT_PE					4
#define USART_ERR_FE					5
#define USART_ERR_NF					6
#define USART_ERR_ORE					7

//USARtx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART_PeriClockControl(USARTx_RegDef_t *pUSARTx, uint8_t EnorDi);

void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USARTx_RegDef_t *pUSARTx);

void USART_SendData(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

void USART_PeripheralControl(USARTx_RegDef_t *pUSARTx, uint8_t EnOrDi);
uint8_t USART_GetFlagStatus(USARTx_RegDef_t *pUSARTx , uint32_t FlagName);
void USART_ClearFlag(USARTx_RegDef_t *pUSARTx, uint16_t StatusFlagName);

void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);
void USART_SetBaudRate(USARTx_RegDef_t *pUSARTx, uint32_t BaudRate);

#endif /* INC_STM32F407XX_USARTX_DRIVER_H_ */
