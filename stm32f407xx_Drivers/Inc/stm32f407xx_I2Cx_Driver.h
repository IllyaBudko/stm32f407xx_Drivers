/*
 * stm32f407xx_I2Cx_Driver.h
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#ifndef INC_STM32F407XX_I2CX_DRIVER_H_
#define INC_STM32F407XX_I2CX_DRIVER_H_

#include "stm32f407xx.h"



typedef struct
{
	volatile uint32_t I2C_SCLSpeed;
	volatile uint8_t  I2C_DeviceAddr;			//user defined
	volatile uint8_t  I2C_ACKCtrl;
	volatile uint8_t  I2C_FMDutyCycle;
}I2Cx_Config_t;

typedef struct
{
	I2Cx_RegDef_t *pI2Cx;
	I2Cx_Config_t I2C_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxRxState;
	uint8_t devAddr;
	uint32_t RxSize;
	uint8_t Sr;
}I2Cx_Handle_t;


//I2Cx application states
#define I2C_READY				0
#define I2C_BUSY_IN_RX			1
#define I2C_BUSY_IN_TX			2

//I2Cx application events macros
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ERROR_BERR 			3
#define I2C_ERROR_ARLO			4
#define I2C_ERROR_AF			5
#define I2C_ERROR_OVR			6
#define I2C_ERROR_TIMEOUT		7

//I2Cx Configuration definitions

//I2Cx Speed configuration macros
#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K		200000

//I2Cx Acknowledge control
#define I2C_ACK_EN				1
#define I2C_ACK_DI				0

//I2Cx Fast mode duty cycle
#define I2C_FMDUTY_2			0
#define I2C_FMDUTY_16_9			1

//SPIx status flag definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2C_FLAG_SB				(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR			(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF			(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10			(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF			(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE			(1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE			(1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR			(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO			(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF				(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR			(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR			(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT		(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT		(1 << I2C_SR1_SMBALERT)

//I2Cx Generic macros
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define I2C_DISABLE_SR			0
#define I2C_ENABLE_SR			1

//I2Cx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_PCtrl(I2Cx_RegDef_t *pI2Cx, uint8_t ENorDI);
void I2C_Pclk_Init(I2Cx_RegDef_t *pI2Cx, uint8_t ENorDI);

void I2C_Init(I2Cx_Handle_t *pI2CHandle);
void I2C_DeInit(I2Cx_RegDef_t *pI2Cx);

uint8_t I2C_GetFlagStatus(I2Cx_RegDef_t *pI2Cx, uint32_t flagName);

void I2C_MasterSendData (I2Cx_Handle_t *pI2Chandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);
void I2C_MasterReceiveData (I2Cx_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);

uint8_t I2C_Master_IT_SendData(I2Cx_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);
uint8_t I2C_Master_IT_ReceiveData(I2Cx_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t slaveAddr, uint8_t Sr);

void I2C_CloseReceiveData (I2Cx_Handle_t *pI2CHandle);
void I2C_CloseSendData (I2Cx_Handle_t *pI2CHandle);

//IRQ Configuration and handling APIs for I2Cx
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2Cx_Handle_t *pI2CHandle);
void I2C_ERR_IRQHandling(I2Cx_Handle_t *pI2CHandle);

//Peripheral control helper APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t I2C_GetFlagStatus(I2Cx_RegDef_t *pI2Cx, uint32_t flagName);
void I2C_AppEvCallback(I2Cx_Handle_t *pI2CHandle, uint8_t AppEv);
void I2C_ManageAck(I2Cx_RegDef_t *pI2Cx, uint8_t ENorDi);



#endif /* INC_STM32F407XX_I2CX_DRIVER_H_ */
