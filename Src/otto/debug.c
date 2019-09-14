/*
 * debug.c
 *
 *  Created on: Jun 13, 2019
 *      Author: Jarosław Królik
 */

//#include "main.h"
#include "limbs.h"
#include "servo.h"
#include <string.h> //strchr
#include <stdlib.h> //strtol

typedef enum{
	DEBUG_OK 	= 0x00U,
	DEBUG_ERROR	= 0x01U
}DebugStatusTypeDef;

DebugStatusTypeDef String_MoveToNextWord(uint8_t **pBuffer);
int32_t Word2Num(uint8_t *word, uint8_t base);

void otto_debugHandler(uint8_t* Buf, uint32_t *Len){
	LimbsNumTypeDef legNum = LegError;

	if( memcmp(Buf,"left ",5) == 0 )
		legNum = LegLeft;

	if( memcmp(Buf,"right ",6) == 0 )
		legNum = LegRight;

	if(legNum == LegError)
		return;

	//move pointer to next word
	if(String_MoveToNextWord(&Buf) != DEBUG_OK)
		return;

	LimbJointNumTypeDef jointNum = JointError;

	if( memcmp(Buf,"ankle ",6) == 0 )
		jointNum = JointAnkle;

	if( memcmp(Buf,"hip ",4) == 0 )
		jointNum = JointHip;

	if(jointNum == JointError)
		return;

	//move pointer to next word
	if(String_MoveToNextWord(&Buf) != DEBUG_OK)
		return;

	int16_t value = (int16_t)Word2Num(Buf,10);
	if(value<-90 || value>90)
		return;

	limbs_setPositon(legNum, jointNum, value);
}

DebugStatusTypeDef String_MoveToNextWord(uint8_t **pBuffer){
	//move pointer to next word
	*pBuffer = (uint8_t*)strchr((char*)*pBuffer, ' ');
	if(pBuffer == 0)
		return DEBUG_ERROR;
	//miss the space symbol
	(*pBuffer)++;
	return DEBUG_OK;
}

inline int32_t Word2Num(uint8_t *word, uint8_t base){
	if(word[1] == 'x')
		base = 16;
	else if(word[1] == 'b'){
		base = 2;
		//miss "0b"
		word += 2;
	}
	return (int32_t)strtol((char*)word, NULL, (int)base);
}
