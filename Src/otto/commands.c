/*
 * commands.c
 *
 *  Created on: Jun 13, 2019
 *      Author: Jarosław Królik
 */

#include "commands.h"

#include "stdint.h" //uint8_t
#include "stdbool.h"

/* Private define ------------------------------------------------------------*/
#define COMMANDS_buffer_size 2

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
void (*COMMANDS_buffer[COMMANDS_buffer_size])();
uint8_t COMMANDS_buffer_head;
uint8_t COMMANDS_buffer_tail;
bool COMMANDS_buffer_empty;
volatile uint8_t COMMANDS_stop;

/* Private function prototypes -----------------------------------------------*/
void commands_clearBuffer();

/* Global function  ----------------------------------------------------------*/
/********************  COMMANDS initialization  *******************/
void commands_init(){
	commands_clearBuffer();
	COMMANDS_stop = false;
}

/********************  COMMANDS handlers  *******************/
 void commands_executNext(){
	if(COMMANDS_stop)
		return;
	//check if buffer is clear
	if(COMMANDS_buffer_tail == COMMANDS_buffer_head)
		return;

	//execute command
	(*COMMANDS_buffer[COMMANDS_buffer_tail])();

	//increment tail
	if(COMMANDS_buffer_tail == COMMANDS_buffer_size - 1)
		COMMANDS_buffer_tail = 0;
	else
		COMMANDS_buffer_tail++;

	//if buffer still is empty, clear flag
	if(COMMANDS_buffer_tail == COMMANDS_buffer_head)
		COMMANDS_buffer_empty = true;
}

void commands_addToBufferHandler(void (*pfunction)()){
	COMMANDS_stop = false;

	//check if there are free space
	//if there are no free space, change last element of buffer (decrement pointer)
	if(COMMANDS_buffer_head == COMMANDS_buffer_tail && !COMMANDS_buffer_empty){
		if(COMMANDS_buffer_head == 0)
			COMMANDS_buffer_head = COMMANDS_buffer_size - 1;
		else
			COMMANDS_buffer_head--;
	}
	COMMANDS_buffer_empty = false;

	//copy data to buffer
	COMMANDS_buffer[COMMANDS_buffer_head] = pfunction;

	//buffer head should be free -> increment head
	if(COMMANDS_buffer_head == COMMANDS_buffer_size - 1)
		COMMANDS_buffer_head = 0;
	else
		COMMANDS_buffer_head++;
}

void commands_stopHandler(){
	COMMANDS_stop = true;

	//clear buffer
	commands_clearBuffer();
}

/* Private function  ---------------------------------------------------------*/
void commands_clearBuffer(){
	COMMANDS_buffer_head = 0;
	COMMANDS_buffer_tail = 0;
	COMMANDS_buffer_empty = true;
}
