/*
 * limbs.c
 *
 *  Created on: Jun 12, 2019
 *      Author: Jarosław Królik
 */
#include "limbs.h"
#include "servo.h"

//LegTypeDef Legs[2];
LegsTypeDef Legs;



void limbs_init(){
	servo_init();

	Legs.left.ankle.servoNum	= LegServoNum_LeftAnkle;
	Legs.left.hip.servoNum		= LegServoNum_LeftHip;
	Legs.right.ankle.servoNum	= LegServoNum_RightAnkle;
	Legs.right.hip.servoNum		= LegServoNum_RightHip;

	Legs.left.ankle.servoDirection	= LegServoDirection_LeftAnkle;
	Legs.left.hip.servoDirection	= LegServoDirection_LeftHip;
	Legs.right.ankle.servoDirection	= LegServoDirection_RightAnkle;
	Legs.right.hip.servoDirection	= LegServoDirection_RightHip;

	servo_set_calc_param(Legs.left.ankle.servoNum,0,30,0);
	servo_set_calc_param(Legs.left.hip.servoNum,0,30,0);
	servo_set_calc_param(Legs.right.ankle.servoNum,0,30,0);
	servo_set_calc_param(Legs.right.hip.servoNum,0,30,0);

	limbs_setHipPositon(LegLeft,0);
	limbs_setHipPositon(LegRight,0);
	limbs_setAnklePositon(LegLeft,0);
	limbs_setAnklePositon(LegRight,0);
}

void limbs_setAnklePositon(LegNumTypeDef legNum, int16_t angle){
	LegTypeDef *leg = limbs_getLeg(legNum);
	if(leg == 0)
		return;
	angle *= leg->ankle.servoDirection;
	servo_set_position(leg->ankle.servoNum, (type_angle)angle);
}

void limbs_setHipPositon(LegNumTypeDef legNum, int16_t angle){
	LegTypeDef *leg = limbs_getLeg(legNum);
	if(leg == 0)
		return;
	angle *= leg->hip.servoDirection;
	servo_set_position(leg->hip.servoNum, (type_angle)angle);
}


LegTypeDef *limbs_getLeg(LegNumTypeDef legNum){
	switch (legNum){
	case LegLeft: return &(Legs.left);
	case LegRight: return &(Legs.right);
	case LegError: return 0;
	}
	return 0;
}
