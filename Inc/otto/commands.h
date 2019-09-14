/*
 * commands.h
 *
 *  Created on: Jun 13, 2019
 *      Author: Jarosław Królik
 */

#ifndef OTTO_COMMANDS_H_
#define OTTO_COMMANDS_H_

//#include "stdint.h" //uint8_t

void commands_init();
void commands_executNext();
void commands_addToBufferHandler(void (*pfunction)());
void commands_stopHandler();

#endif /* OTTO_COMMANDS_H_ */
