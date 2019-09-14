/*
 * bluetooth.c
 *
 *  Created on: Sep 13, 2019
 *      Author: Jarosław Królik
 */

#include "bluetooth.h"
#include "usart.h"

#include "control_init.h" //OTTO_StatusTypeDef
#include <string.h> //strchr
#include <stdlib.h> //strtol

/* Private define ------------------------------------------------------------*/
#define UART_RX_buffer_size 30
#define UART_TX_buffer_size 80
#define UART_TX_buffer_lines 6

/* Private typedef -----------------------------------------------------------*/
typedef struct{
	uint8_t line[UART_TX_buffer_size];
	uint8_t size;
}UART_TX_buffer_line;

/* Private variables ---------------------------------------------------------*/
uint8_t UART_RX_buffer[UART_RX_buffer_size];
UART_TX_buffer_line UART_TX_buffer[UART_TX_buffer_lines];
uint8_t UART_TX_buffer_head;
uint8_t UART_TX_buffer_tail;
uint8_t UART_TX_fast_send;

/* Private function prototypes -----------------------------------------------*/
void bluetooth_SendBufferElement(uint8_t *pData, uint16_t Size);
void bluetooth_SendCalcLenght(char *pData);

OTTO_StatusTypeDef bluetooth_ReceiveDecode();

/********************  USART initialization  *******************/
void bluetooh_Init(void){
	//MX_USARTx_UART_Init() - was called

	UART_TX_buffer_head = 0;
	UART_TX_buffer_tail = 0;
	UART_TX_fast_send = 0;

	//start listening
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}

/********************  USART interrupt - finished send buffer element *******************/
void bluetooth_TransmitEndHandler(){
	if(UART_TX_fast_send){
		UART_TX_fast_send = 0;
	}else{
		//increment tail
		if(UART_TX_buffer_tail == UART_TX_buffer_lines - 1)
			UART_TX_buffer_tail = 0;
		else
			UART_TX_buffer_tail++;
	}

	//if current tail has not been send, send it
	if(UART_TX_buffer_tail != UART_TX_buffer_head)
		bluetooth_SendBufferElement(&(UART_TX_buffer[UART_TX_buffer_tail].line[0]), UART_TX_buffer[UART_TX_buffer_tail].size);
}

/********************  USART interrupt - received data  *******************/
void bluetooth_ReceiveHandler(){
	OTTO_StatusTypeDef Status = bluetooth_ReceiveDecode();
	if(Status == OTTO_OK);
}

/********************  USART send data  *******************/
void bluetooth_Send(uint8_t *pData, uint16_t Size){
	//if usart is free send direct data
	HAL_UART_StateTypeDef uartState = HAL_UART_GetState(USART_BLUETOOTH);

	if(UART_TX_buffer_tail == UART_TX_buffer_head && uartState == (HAL_UART_STATE_READY | HAL_UART_STATE_BUSY_RX)){
		UART_TX_fast_send = 1;
		bluetooth_SendBufferElement(pData,Size);
		return;
	}
	//copy data to buffer
	UART_TX_buffer[UART_TX_buffer_head].size = Size;
	uint8_t *pTemp = &(UART_TX_buffer[UART_TX_buffer_head].line[0]);
	for(uint8_t i=0; i<Size; i++){
		*pTemp = *pData;
		pTemp++;
		pData++;
	}

	//increment head -> new element arrived
	if(UART_TX_buffer_head == UART_TX_buffer_lines - 1)
		UART_TX_buffer_head = 0;
	else
		UART_TX_buffer_head++;
}

/********************  USART private function  *******************/
void bluetooth_SendBufferElement(uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit_IT(USART_BLUETOOTH, pData, Size);

	//turn on listen after transmit
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}

void bluetooth_SendCalcLenght(char *pData){
	uint16_t size = strlen(pData);
	HAL_UART_Transmit_IT(USART_BLUETOOTH, (uint8_t*)pData, size);

	//turn on listen after transmit
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}

OTTO_StatusTypeDef bluetooth_ReceiveDecode(){
	/*uint8_t *pDebugRxBuffer = (uint8_t *)UART_RX_buffer;

	if( memcmp(pDebugRxBuffer," ",7) == 0 ){
		//move pointer to next word
		if(String_MoveToNextWord(&pDebugRxBuffer) != PD_OK){
			bluetooth_SendCalcLenght("Can't find parameter after 'Control'\n\r");
			return PD_ERROR;
		}
		return bluetooth_ReceiveDecode_Control(pDebugRxBuffer);
	}
	if( memcmp(pDebugRxBuffer,"Unitek ",7) == 0 ){
		//move pointer to next word
		if(String_MoveToNextWord(&pDebugRxBuffer) != PD_OK){
			bluetooth_SendCalcLenght("Can't find parameter after 'Unitek'\n\r");
			return PD_ERROR;
		}
		return bluetooth_ReceiveDecode_Unitek(pDebugRxBuffer);
	}*/
	return OTTO_ERROR;
}
