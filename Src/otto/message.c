/*
 * message.c
 *
 *  Created on: Sep 14, 2019
 *      Author: Jarosław Królik
 */

/* Private define ------------------------------------------------------------*/
#include "message.h"
#include "commands.h"
#include "move.h"

#include <string.h> //strchr
#include <stdlib.h> //strtol

/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
OTTO_StatusTypeDef message_findNextMessage(uint8_t **ppMesageBuffer);
uint8_t message_lenght(uint8_t *pMesageBuffer);

/* Global function  -----------------------------------------------*/
OTTO_StatusTypeDef message_handler(uint8_t *pMesageBuffer){
	OTTO_StatusTypeDef status = message_findNextMessage(&pMesageBuffer);
	if (status != OTTO_OK)
		return status;

	while( status == OTTO_OK ){
		if( memcmp(pMesageBuffer,"<stop>",6) == 0 ){

		}else if( memcmp(pMesageBuffer,"<ruch_tyl>",10) == 0 ){

		}else if( memcmp(pMesageBuffer,"<ruch_lewo>",11) == 0 ){

		}else if( memcmp(pMesageBuffer,"<ruch_prawo>",12) == 0 ){

		}else if( memcmp(pMesageBuffer,"<ruch_przod>",12) == 0 ){
			commands_addToBufferHandler(&move_przod);
		}else if( memcmp(pMesageBuffer,"<ruch_akcja>",12) == 0 ){

		}
		//move to next command
		pMesageBuffer++;
		status = message_findNextMessage(&pMesageBuffer);
	}
	return OTTO_OK;
}

/* Private function  -----------------------------------------------*/
OTTO_StatusTypeDef message_findNextMessage(uint8_t **ppMesageBuffer){
	//move pointer to next word
	*ppMesageBuffer = (uint8_t*)strchr((char*)*ppMesageBuffer, '<');
	if(*ppMesageBuffer == 0)
		return OTTO_ERROR;
	return OTTO_OK;
}

uint8_t message_lenght(uint8_t *pMesageBuffer){
	uint8_t *pMessageStart = pMesageBuffer;
	//find message end
	pMesageBuffer = (uint8_t*)strchr((char*)pMesageBuffer, '>');

	if(pMesageBuffer == 0)
		return 0;
	return pMesageBuffer - pMessageStart;
}

