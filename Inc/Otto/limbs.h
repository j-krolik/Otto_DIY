/*
 * limbs.h
 *
 *  Created on: Jun 12, 2019
 *      Author: Jarosław Królik
 */

#ifndef OTTO_LIMBS_H_
#define OTTO_LIMBS_H_

#include "stdint.h" //uint8_t

typedef enum{
	LimbOK		= 0x0u,
	LimbBusy	= 0x1u,
	LimbError	= 0x2u
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

typedef enum{
	LimbSpeedVeryFast	= 0x0u,
	LimbSpeedFast		= 0x1u,
	LimbSpeedNormal		= 0x2u,
	LimbSpeedSlow		= 0x3u,
	LimbSpeedVerySlow	= 0x4u
}LimbSpeedTypeDef;

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
void limbs_setPositonSingle(LegNumTypeDef legNum, JointNumTypeDef jointNum, int16_t angle);
LimbStatusTypeDef limbs_getStatus(LegNumTypeDef legNum, JointNumTypeDef jointNum);
void limbs_changeSpeed(LegNumTypeDef legNum, JointNumTypeDef jointNum, LimbSpeedTypeDef speed);
void limbs_changeSpeedPercentage(LegNumTypeDef legNum, JointNumTypeDef jointNum, int8_t PercentageOfAlphaMax, int8_t PercentageOfOmegaMax);

#endif /* OTTO_LIMBS_H_ */
