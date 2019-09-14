/*
 * bluetooth.c
 *
 *  Created on: Sep 13, 2019
 *      Author: Jarosław Królik
 */

#include "bluetooth.h"
#include "usart.h"

#include "control_init.h" //OTTO_StatusTypeDef
#include "message.h"	//message_handler
#include <string.h> //strlen

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
void bluetooth_sendBufferElement(uint8_t *pData, uint16_t Size);
void bluetooth_sendCalcLenght(char *pData);

/********************  USART initialization  *******************/
void bluetooh_init(void){
	//MX_USARTx_UART_Init() - was called

	UART_TX_buffer_head = 0;
	UART_TX_buffer_tail = 0;
	UART_TX_fast_send = 0;

	//start listening
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}

/********************  USART interrupt - finished send buffer element *******************/
void bluetooth_transmitEndHandler(){
	if(UART_TX_fast_send){
		UART_TX_fast_send = 0;
	}else{
		//increment tail
		if(UART_TX_buffer_tail == UART_TX_buffer_lines - 1)
			UART_TX_buffer_tail = 0;
		else
			UART_TX_buffer_tail++;
	}

	//if buffer isn't empty, send next element
	if(UART_TX_buffer_tail != UART_TX_buffer_head)
		bluetooth_sendBufferElement(&(UART_TX_buffer[UART_TX_buffer_tail].line[0]), UART_TX_buffer[UART_TX_buffer_tail].size);
}

/********************  USART interrupt - received data  *******************/
void bluetooth_receiveHandler(){
	//decode message
	message_handler( (uint8_t *)UART_RX_buffer );

	//listening has benn stoped -> start listening
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}

/********************  USART send data  *******************/
void bluetooth_send(uint8_t *pData, uint16_t Size){
	//if usart is free send direct data
	HAL_UART_StateTypeDef uartState = HAL_UART_GetState(USART_BLUETOOTH);

	if(UART_TX_buffer_tail == UART_TX_buffer_head && uartState == (HAL_UART_STATE_READY | HAL_UART_STATE_BUSY_RX)){
		UART_TX_fast_send = 1;
		bluetooth_sendBufferElement(pData,Size);
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
void bluetooth_sendBufferElement(uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit_IT(USART_BLUETOOTH, pData, Size);

	//turn on listen after transmit
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}

void bluetooth_sendCalcLenght(char *pData){
	uint16_t size = strlen(pData);
	HAL_UART_Transmit_IT(USART_BLUETOOTH, (uint8_t*)pData, size);

	//turn on listen after transmit
	HAL_UART_Receive_IT(USART_BLUETOOTH, UART_RX_buffer, UART_RX_buffer_size);
}
