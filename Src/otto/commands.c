/*
 * commands.c
 *
 *  Created on: Jun 13, 2019
 *      Author: Jarosław Królik
 */

#include "commands.h"
#include "limbs.h"


void commands_step(int8_t ankle1A, int8_t ankle1B, int8_t hip2){
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
