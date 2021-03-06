/*
 * stm32f407xx_SPIx_Driver.h
 *
 *  Created on: Jan 18, 2020
 *      Author: Illya Budko
 */

#ifndef INC_STM32F407XX_SPIX_DRIVER_H_
#define INC_STM32F407XX_SPIX_DRIVER_H_

#include "stm32f407xx.h"

//SPIx register definition structures
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SPIx pin configuration definition structure
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SClkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPIx_Config_t;

//SPIx handle definition structure
typedef struct
{
	SPIx_RegDef_t *pSPIx;
	SPIx_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPIx_Handle_t;

//SPIx Configuration definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SPIx application state definitions
#define SPI_READY				0
#define SPI_BUSY_RX				1
#define SPI_BUSY_TX				2

//SPIx application events definitions
#define SPI_EVENT_CMPLT_TX		1
#define SPI_EVENT_CMPLT_RX		2
#define SPI_EVENT_OVR_ERR		3
#define SPI_EVENT_CRC_ERR		4

//SPIx device mode definitions
#define SPI_MODE_MASTER			1
#define SPI_MODE_SLAVE			0

//SPIx bus configuration definitions
#define SPI_BUS_CONFIG_FD		1
#define SPI_BUS_CONFIG_HD		2
#define SPI_BUS_CONFIG_SX_RX	3

//SPIx serial clock speed definitions
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

//SPIx data format frame definitions
#define SPI_DFF_8BIT			0
#define SPI_DFF_16BIT			1

//SPIx serial clock polarity definitions
#define SPI_CPOL_HIGH			1
#define SPI_CPOL_LOW			0

//SPIx serial clock phase definitions
#define SPI_CPHA_HIGH			1
#define SPI_CPHA_LOW			0

//SPIx software slave management definitions
#define SPI_SSM_EN				1
#define SPI_SSM_DI				0

//SPIx status flag definitions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define SPI_FLAG_RXNE		(1 << SPI_SR_RXNE)
#define SPI_FLAG_TXE		(1 << SPI_SR_TXE)
#define SPI_FLAG_CHSIDE		(1 << SPI_SR_CHSIDE)
#define SPI_FLAG_UDR		(1 << SPI_SR_UDR)
#define SPI_FLAG_CRCERR		(1 << SPI_SR_CRCERR)
#define SPI_FLAG_MODF		(1 << SPI_SR_MODF)
#define SPI_FLAG_OVR		(1 << SPI_SR_OVR)
#define SPI_FLAG_BSY		(1 << SPI_SR_BSY)
#define SPI_FLAG_FRE		(1 << SPI_SR_FRE)

//SPIx APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SPI_PCtrl(SPIx_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_Pclk_Init(SPIx_RegDef_t *pSPIx, uint8_t ENorDI);
void SPI_Init(SPIx_Handle_t *pSPIHandle);
void SPI_DeInit(SPIx_RegDef_t *pSPIx);

void SPI_SendData(SPIx_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData (SPIx_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_IT_SendData(SPIx_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_IT_ReceiveData (SPIx_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

//IRQ Configuration and handling APIs for SPIx
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPIx_Handle_t *pSPIHandle);

//Peripheral control helper APIs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t SPI_GetFlagStatus(SPIx_RegDef_t *pSPIx, uint32_t flagName);
void SPI_Clear_OVRFlag(SPIx_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPIx_Handle_t *pSPIHandle);
void SPI_CloseReception(SPIx_Handle_t *pSPIHandle);
void SPI_AppEvCallback(SPIx_Handle_t *pSPIHandle, uint8_t AppEv);
void  SPI_SSIConfig(SPIx_RegDef_t *pSPIx, uint8_t EnOrDi);
void  SPI_SSOEConfig(SPIx_RegDef_t *pSPIx, uint8_t EnOrDi);




#endif /* INC_STM32F407XX_SPIX_DRIVER_H_ */
