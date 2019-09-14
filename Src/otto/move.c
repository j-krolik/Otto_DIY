/*
 * move.c
 *
 *  Created on: Sep 14, 2019
 *      Author: Jarosław Królik
 */

#include "move.h"

#include "stdint.h" //uint8_t
#include "limbs.h"

/* Private define ------------------------------------------------------------*/
#define MOVE_buffer_size 2

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void move_step(int8_t ankle1A, int8_t ankle1B, int8_t hip2);

/* Global function  ----------------------------------------------------------*/
void move_przod(){
	int8_t ankle1A = -18;
	int8_t ankle1B = 6;
	int8_t Hip2A = 25;
	int8_t Hip2B = -Hip2A;
	int8_t ankle3A = -18;
	int8_t ankle3B = 6;

	move_step(ankle1A, ankle1B, Hip2A);
	//move_step(ankle3B, ankle3A, Hip2B);
	//move_step(ankle3A, ankle3B, Hip2A);
	move_step(ankle3B, ankle3A, 0);
	move_step(0, 0, 0);

	move_step(ankle1B, ankle1A, Hip2B);
	//move_step(ankle3A, ankle3B, Hip2A);
	//move_step(ankle3B, ankle3A, Hip2B);
	move_step(ankle3A, ankle3B, 0);
	move_step(0, 0, 0);
}

/* Private function  ---------------------------------------------------------*/
void move_step(int8_t ankle1A, int8_t ankle1B, int8_t hip2){
	//half step
	limbs_setPositonMulti(LegRight, JointAnkle, ankle1A);
	limbs_setPositonMulti(LegLeft, JointAnkle, ankle1B);
	while( limbs_getStatusMulit(LegLeft|LegRight, JointAnkle) == LimbBusy);
	//HAL_Delay(1000);
	limbs_setPositonMulti(LegRight, JointHip, hip2);
	limbs_setPositonMulti(LegLeft, JointHip, -hip2);
	while( limbs_getStatusMulit(LegLeft|LegRight, JointHip) == LimbBusy);
	//HAL_Delay(1000);
}

