#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "main.h"
#include "cmsis_os.h"

#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0

/* ----------------------- Static variables ---------------------------------*/
UART_HandleTypeDef *uartM;
static uint8_t singlechar;

/* ----------------------- User defenitions ---------------------------------*/
//#define RS485_MASTER_RTS_LOW	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port, RS485_RTS_Pin, GPIO_PIN_RESET)
//#define RS485_MASTER_RTS_HIGH 	HAL_GPIO_WritePin(RS485_RTS_GPIO_Port, RS485_RTS_Pin, GPIO_PIN_SET)

/* ----------------------- Start implementation -----------------------------*/

BOOL xMBMasterPortSerialInit(void *dHUART, ULONG ulBaudRate, void *dHTIM)
{
	uartM = (UART_HandleTypeDef *)dHUART;

	return TRUE;
}

void vMBMasterPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
	if(xRxEnable)
	{
		taskENTER_CRITICAL_FROM_ISR();
		//RS485_MASTER_RTS_LOW;
		HAL_UART_Receive_IT(uartM, &singlechar, 1);
		taskEXIT_CRITICAL_FROM_ISR(0);

	}	
	else
	{
		HAL_UART_AbortReceive_IT(uartM);

	}

	if(xTxEnable)
	{
		//RS485_MASTER_RTS_HIGH;
		pxMBMasterFrameCBTransmitterEmpty();
	}
	else
	{
		taskENTER_CRITICAL_FROM_ISR();
		HAL_UART_AbortTransmit_IT(uartM);
		taskEXIT_CRITICAL_FROM_ISR(0);
	}
}

void vMBMasterPortClose(void)
{
	HAL_UART_AbortReceive_IT(uartM);
	HAL_UART_AbortTransmit_IT(uartM);
}

BOOL xMBMasterPortSerialPutBytes(volatile UCHAR *ucByte, USHORT usSize)
{
	taskENTER_CRITICAL_FROM_ISR();
	HAL_UART_Transmit_IT(uartM, (uint8_t *)ucByte, usSize);
	taskEXIT_CRITICAL_FROM_ISR(0);
	return TRUE;
}

BOOL xMBMasterPortSerialPutByte(CHAR ucByte)
{
	taskENTER_CRITICAL_FROM_ISR();
	HAL_UART_Transmit_IT(uartM, (uint8_t*)&ucByte, 1);
	//extern UART_HandleTypeDef huart2;//////////////
	//HAL_UART_Transmit(&huart2, "HERE", 4, HAL_MAX_DELAY);///////
	taskEXIT_CRITICAL_FROM_ISR(0);
	return TRUE;
}

BOOL xMBMasterPortSerialGetByte(CHAR * pucByte)
{
	*pucByte = (uint8_t)(singlechar);
	return TRUE;
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == uartM->Instance)
//	{
//		taskENTER_CRITICAL_FROM_ISR();
//		pxMBMasterFrameCBByteReceived();
//		HAL_UART_Receive_IT(uartM, &singlechar, 1);
//		taskEXIT_CRITICAL_FROM_ISR(0);
//
//	}
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == uartM->Instance)
//	{
//		taskENTER_CRITICAL_FROM_ISR();
//		pxMBMasterFrameCBTransmitterEmpty();
//		taskEXIT_CRITICAL_FROM_ISR(0);
//	}
//}

#endif
