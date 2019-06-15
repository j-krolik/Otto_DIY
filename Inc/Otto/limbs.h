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
}LimbsNumTypeDef;

typedef enum{
	JointAnkle	= 0x1u,
	JointHip	= 0x2u,
	JointError	= 0x0u
}LimbJointNumTypeDef;

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
}LimbsJointypeDef;

typedef struct{
	LimbsJointypeDef ankle;
	LimbsJointypeDef hip;
}LimbsLegTypeDef;

typedef struct{
	LimbsLegTypeDef left;
	LimbsLegTypeDef right;
}LimbsTypeDef;

#define LEG_SERVO_NUM_LEFT_ANKLE	2
#define LEG_SERVO_NUM_LEFT_HIP		1
#define LEG_SERVO_NUM_RIGHT_ANKLE	0
#define LEG_SERVO_NUM_RIGHT_HIP		3

#define LEG_SERVO_DIRECTION_LEFT_ANKLE	-1
#define LEG_SERVO_DIRECTION_LEFT_HIP	-1
#define LEG_SERVO_DIRECTION_RIGHT_ANKLE	 1
#define LEG_SERVO_DIRECTION_RIGHT_HIP	 1

#define PARAMETER_SET_DEFAULT -1
#define PARAMETER_DONT_CHANGE  0

void limbs_init();
void limbs_setPositon(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, int16_t angle);
void limbs_setPositonSingle(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, int16_t angle);
LimbStatusTypeDef limbs_getStatus(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum);
void limbs_changeSpeed(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, LimbSpeedTypeDef speed);
void limbs_changeSpeedPercentage(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, int8_t PercentageOfAlphaMax, int8_t PercentageOfOmegaMax);

#endif /* OTTO_LIMBS_H_ */
