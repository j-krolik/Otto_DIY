/*
 * bluetooth.h
 *
 *  Created on: Sep 14, 2019
 *      Author: Jarosław Królik
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "stdint.h" // include data types

void bluetooh_init(void);
void bluetooth_transmitEndHandler(void);
void bluetooth_receiveHandler(void);
void bluetooth_send(uint8_t *pData, uint16_t Size);

#endif /* BLUETOOTH_H_ */
