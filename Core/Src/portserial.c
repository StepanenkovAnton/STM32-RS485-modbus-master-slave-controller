#include "port.h"
#include "mb.h"
#include "mbport.h"
#include "main.h"
#include "cmsis_os.h"

#if MB_SLAVE_RTU_ENABLED > 0 || MB_SLAVE_ASCII_ENABLED > 0

/* ----------------------- Static variables ---------------------------------*/
UART_HandleTypeDef *uart;
extern UART_HandleTypeDef *uartM;
static uint8_t singlechar;


extern uint8_t LedTxRxOn;

/* ----------------------- User defenitions ---------------------------------*/
#define RS485_RTS_LOW	HAL_GPIO_WritePin(RS_485_DE_GPIO_Port, RS_485_DE_Pin, GPIO_PIN_RESET)
#define RS485_RTS_HIGH 	HAL_GPIO_WritePin(RS_485_DE_GPIO_Port, RS_485_DE_Pin, GPIO_PIN_SET)

#define RS485_RE_LOW	HAL_GPIO_WritePin(RS_485_RE_GPIO_Port, RS_485_RE_Pin, GPIO_PIN_RESET)
#define RS485_RE_HIGH 	HAL_GPIO_WritePin(RS_485_RE_GPIO_Port, RS_485_RE_Pin, GPIO_PIN_SET)


/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit( void *dHUART, ULONG ulBaudRate, void *dHTIM )
{
	uart = (UART_HandleTypeDef *)dHUART;

	return TRUE;
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
	if(xRxEnable)
	{
		taskENTER_CRITICAL_FROM_ISR();
		RS485_RTS_LOW;
		RS485_RE_LOW;
		HAL_UART_Receive_IT(uart, &singlechar, 1);
		taskEXIT_CRITICAL_FROM_ISR(0);

	}	
	else
	{
		HAL_UART_AbortReceive_IT(uart);

	}

	if(xTxEnable)
	{
		RS485_RTS_HIGH;
		RS485_RE_HIGH;
		pxMBFrameCBTransmitterEmpty();
	}
	else
	{
		taskENTER_CRITICAL_FROM_ISR();
		HAL_UART_AbortTransmit_IT(uart);
		taskEXIT_CRITICAL_FROM_ISR(0);
	}
}

void vMBPortClose(void)
{
	HAL_UART_AbortReceive_IT(uart);
	HAL_UART_AbortTransmit_IT(uart);
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
	taskENTER_CRITICAL_FROM_ISR();
	HAL_UART_Transmit_IT(uart, (uint8_t*)&ucByte, 1);
	taskEXIT_CRITICAL_FROM_ISR(0);
	return TRUE;
}

BOOL xMBPortSerialPutBytes(volatile UCHAR *ucByte, USHORT usSize)
{
	taskENTER_CRITICAL_FROM_ISR();
	HAL_UART_Transmit_IT(uart, (uint8_t *)ucByte, usSize);
	//HAL_UART_Transmit_DMA(uart, (uint8_t *)ucByte, usSize);
	taskEXIT_CRITICAL_FROM_ISR(0);
	return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
	*pucByte = (uint8_t)(singlechar);
	return TRUE;
	
}

//in case of a noise on the RS485 input (for example, when one modbus device is being reflashed @115200, others recieve
//the communication at 115200 as a noise as they're using the 19200 speed) the device may hang while recieveng a non-interruptive
//sequence of bytes.
//To avoid this, the DMA re-initialization is issued on uart error
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	//__HAL_UART_CLEAR_PEFLAG(huart);
	//HAL_UART_DMAStop(huart);
	//HAL_UART_MspDeInit(huart);
	//HAL_UART_MspInit(huart);
	//HAL_UART_Receive_DMA(uart, &singlechar, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	taskENTER_CRITICAL_FROM_ISR();	
	if (huart->Instance == uart->Instance)
	{
		pxMBFrameCBByteReceived();
		HAL_UART_Receive_IT(uart, &singlechar, 1);//more safe, but interrupts RTOS tasks
	}

		
	if (huart->Instance == uartM->Instance)
	{
		pxMBMasterFrameCBByteReceived();
		HAL_UART_Receive_IT(uartM, &singlechar, 1);

	}

	taskEXIT_CRITICAL_FROM_ISR(0);
	//LedTxRxOn = 10;
	//extern UART_HandleTypeDef huart2;
	//HAL_UART_Transmit(&huart2, (uint8_t*)"HERE", strlen("HERE"), HAL_MAX_DELAY);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == uart->Instance)
	{
		
		taskENTER_CRITICAL_FROM_ISR();
		pxMBFrameCBTransmitterEmpty();
		taskEXIT_CRITICAL_FROM_ISR(0);
		//LedTxRxOn = 10;
	}
	
	if (huart->Instance == uartM->Instance)
	{
		taskENTER_CRITICAL_FROM_ISR();
		pxMBMasterFrameCBTransmitterEmpty();
		taskEXIT_CRITICAL_FROM_ISR(0);
	}

}

#endif
