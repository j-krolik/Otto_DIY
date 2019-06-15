/*
 * limbs.h
 *
 *  Created on: Jun 12, 2019
 *      Author: Jarosław Królik
 */

#ifndef OTTO_LIMBS_H_
#define OTTO_LIMBS_H_

#include "stdint.h" //uint8_t
#include "servo.h"

typedef enum{
	LimbOK		= 0x0u,
	LimbError	= 0x1u,
	LimbBusy	= 0x2u
}LimbStatusTypeDef;

typedef enum{
	LegLeft  = 0x1u,
	LegRight = 0x2u,
	LegError = 0x0u
}LegNumTypeDef;

typedef enum{
	JointAnkle	= 0x1u,
	JointHip	= 0x2u,
	JointError	= 0x0u
}JointNumTypeDef;

typedef struct{
	uint8_t servoNum;
	int8_t servoDirection;
}LegJointypeDef;

typedef struct{
	LegJointypeDef ankle;
	LegJointypeDef hip;
}LegTypeDef;

typedef struct{
	LegTypeDef left;
	LegTypeDef right;
}LegsTypeDef;

#define LegServoNum_LeftAnkle	2
#define LegServoNum_LeftHip		1
#define LegServoNum_RightAnkle	0
#define LegServoNum_RightHip	3

#define LegServoDirection_LeftAnkle	-1
#define LegServoDirection_LeftHip	-1
#define LegServoDirection_RightAnkle 1
#define LegServoDirection_RightHip	 1

#define PARAMETER_SET_DEFAULT -1
#define PARAMETER_DONT_CHANGE  0

void limbs_init();
void limbs_setPositon(LegNumTypeDef legNum, JointNumTypeDef jointNum, int16_t angle);
void limbs_changeServoParameters(LegNumTypeDef legNum, JointNumTypeDef JointNum, type_n n_min, type_alpha alpha_max, type_omega omega_max);

#endif /* OTTO_LIMBS_H_ */
