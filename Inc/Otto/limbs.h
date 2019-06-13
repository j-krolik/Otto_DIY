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
	LegLeft  = 0x1u,
	LegRight = 0x2u,
	LegError = 0x3u
}LegNumTypeDef;

typedef enum{
	JoinAnkle = 0x0u,
	JoinHip	  = 0x1u,
	JoinError = 0x2u,
}JointNumTypeDef;

typedef struct{
	uint8_t servoNum;
	int8_t servoDirection;
}LegServoTypeDef;

typedef struct{
	LegServoTypeDef ankle;
	LegServoTypeDef hip;
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

void limbs_init();
void limbs_setAnklePositon(LegNumTypeDef legNum, int16_t angle);
void limbs_setHipPositon(LegNumTypeDef legNum, int16_t angle);

//debug
LegTypeDef *limbs_getLeg(LegNumTypeDef legNum);

#endif /* OTTO_LIMBS_H_ */
