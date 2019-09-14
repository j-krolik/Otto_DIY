/*
 * control_init.h
 *
 *  Created on: Sep 14, 2019
 *      Author: Jarosław Królik
 */

#ifndef OTTO_CONTROL_INIT_H_
#define OTTO_CONTROL_INIT_H_

typedef enum{
  OTTO_OK       = 0x00U,
  OTTO_ERROR    = 0x01U,
  OTTO_BUSY     = 0x02U,
  OTTO_TIMEOUT  = 0x03U,
  OTTO_RESET	  = 0x04U,
} OTTO_StatusTypeDef;

#endif /* OTTO_CONTROL_INIT_H_ */
