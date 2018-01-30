#include <string.h>

#include "uart.h"

#define UART_PACKET_OK 0
#define UART_PACKET_TOO_LONG 1
#define UART_PACKET_CORRUPT 2
#define UART_BUSY_FLAG 1

volatile uint8_t mUartTxBusyFlag = 0;

#define UART_TX_MAXBUF_SZ 64
#define UART_RX_MAXBUF_SZ 64

static char mUartRxString[UART_RX_MAXBUF_SZ];

static uint8_t mUartRxBuffer = '\000';
static uint8_t mUartNewMessage = 0;

static char mUartTxbuffer[UART_TX_MAXBUF_SZ+1];
static int mUartTxBufSz;
	
HAL_StatusTypeDef UART_WaitIfBusy( void );

HAL_StatusTypeDef UART_WaitIfBusy() {
	for (int i = 0; mUartTxBusyFlag && i < 100; i++)
		HAL_Delay(1);
	
	if (mUartTxBusyFlag)
		return HAL_ERROR;
	
	mUartTxBusyFlag  = 1;
	
	return HAL_OK;
}

void UART_Send(UART_HandleTypeDef *huart, const char message[])
{
	if (UART_WaitIfBusy() != HAL_OK)
		return;
		
	mUartTxBufSz = strlen(message);
	if (mUartTxBufSz > UART_TX_MAXBUF_SZ)
		mUartTxBufSz = UART_TX_MAXBUF_SZ-1;
		
	strncpy(mUartTxbuffer, message, mUartTxBufSz);

	if (strlen(message) > UART_TX_MAXBUF_SZ) {
		mUartTxbuffer[UART_TX_MAXBUF_SZ-2] = '\r';
		mUartTxbuffer[UART_TX_MAXBUF_SZ-1] = '\n';
		mUartTxbuffer[UART_TX_MAXBUF_SZ] = '\0';
	}
	
	HAL_UART_Transmit_IT(huart, (uint8_t*)mUartTxbuffer, mUartTxBufSz);
}

void UART_SendChar( UART_HandleTypeDef *huart, int ch ) {
	if (UART_WaitIfBusy() != HAL_OK)
		return;
	
	mUartTxBufSz = 1;
	mUartTxbuffer[0] = (char) ch;
	
	HAL_UART_Transmit_IT(huart, (uint8_t*)mUartTxbuffer, mUartTxBufSz);
}
	

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//	for(int i = 0; i < 10; i++) {
//		BSP_LED_Toggle(LED3);
//		BSP_LED_Toggle(LED4);
//		BSP_LED_Toggle(LED5);
//		BSP_LED_Toggle(LED6);
//		HAL_Delay(100);
//	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	mUartTxBusyFlag  = 0;
	// BSP_LED_Toggle(LED4);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static short int tUartRxIndex = 0;	
	static uint8_t tUartErrorFlag = UART_PACKET_OK;
	
	// if(UartHandle->Instance==USART1){
//	BSP_LED_Toggle(LED4);
//	BSP_LED_Toggle(LED6);
	
	if (mUartRxBuffer == '\n') // If Enter
	{
		if (tUartRxIndex && tUartErrorFlag == UART_PACKET_OK)
		{
			mUartRxString[tUartRxIndex] = '\000';
			tUartRxIndex = 0;
			mUartNewMessage  = 1;
		
		} else {
			// UART_Send("ERROR: UART1 packet too long\r\n");
			tUartErrorFlag = UART_PACKET_OK; // reset error state
		}

	} else {

		if (mUartRxBuffer != '\r' && tUartErrorFlag == UART_PACKET_OK) // Ignore return
		{
			mUartRxString[tUartRxIndex] = mUartRxBuffer; // Add that character to the string
			tUartRxIndex++;
			
			if (tUartRxIndex >= UART_RX_MAXBUF_SZ) // User typing too much, we can't have commands that big
			{
				tUartErrorFlag = UART_PACKET_TOO_LONG;
				tUartRxIndex = 0;
				mUartRxString[tUartRxIndex] = '\000';
			}
		}	
	}

	HAL_UART_Receive_IT(huart, &mUartRxBuffer, 1);
}

