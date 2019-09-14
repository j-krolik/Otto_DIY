/*
 * message.h
 *
 *  Created on: Sep 14, 2019
 *      Author: Jarosław Królik
 */

#ifndef MESSAGE_H_
#define MESSAGE_H_

#include "control_init.h"
#include "stdint.h" //uint8_t

OTTO_StatusTypeDef message_handler(uint8_t *pMesageBuffer);

#endif /* MESSAGE_H_ */
