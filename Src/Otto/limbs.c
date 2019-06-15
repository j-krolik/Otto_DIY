/*
 * limbs.c
 *
 *  Created on: Jun 12, 2019
 *      Author: Jarosław Królik
 */
#include "limbs.h"
#include "servo.h"

void limbs_setPositon_legSingle(LegNumTypeDef *legNum, JointNumTypeDef *jointNum, int16_t *angle);
void limbs_setPositon_legJointSingle(LegNumTypeDef *legNum, JointNumTypeDef *jointNum, int16_t *angle);

void limbs_setServoParameters(LegNumTypeDef *legNum, JointNumTypeDef *jointNum, type_alpha *alpha_max, type_omega *omega_max);

LimbStatusTypeDef limbs_getJoint(LegJointypeDef **joint, LegNumTypeDef *legNum, JointNumTypeDef *jointNum);
LimbStatusTypeDef limbs_getJointFromLeg(LegJointypeDef **joint, LegTypeDef *leg, JointNumTypeDef *jointNum);
LimbStatusTypeDef limbs_getLeg(LegTypeDef **leg, LegNumTypeDef *legNum);

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

	limbs_changeSpeed(LegLeft,	JointAnkle,	LimbSpeedVeryFast);
	limbs_changeSpeed(LegLeft,	JointHip,	LimbSpeedVeryFast);
	limbs_changeSpeed(LegRight,	JointAnkle,	LimbSpeedVeryFast);
	limbs_changeSpeed(LegRight,	JointHip,	LimbSpeedVeryFast);

	limbs_setPositon(LegLeft|LegRight, JointAnkle|JointHip, 0);
}

void limbs_setPositon(LegNumTypeDef legNum, JointNumTypeDef jointNum, int16_t angle){
	LegNumTypeDef legTemp;

	if(legNum & LegLeft){
		legTemp = LegLeft;
		limbs_setPositon_legSingle(&legTemp,&jointNum,&angle);
	}
	if(legNum & LegRight){
		legTemp = LegRight;
		limbs_setPositon_legSingle(&legTemp,&jointNum,&angle);
	}
}

void limbs_setPositonSingle(LegNumTypeDef legNum, JointNumTypeDef jointNum, int16_t angle){
	limbs_setPositon_legJointSingle(&legNum, &jointNum,&angle);
}

void limbs_setPositon_legSingle(LegNumTypeDef *legNum, JointNumTypeDef *jointNum, int16_t *angle){
	JointNumTypeDef jointTemp;

	if(*jointNum & JointAnkle){
		jointTemp = JointAnkle;
		limbs_setPositon_legJointSingle(legNum,&jointTemp,angle);
	}
	if(*jointNum & JointHip){
		jointTemp = JointHip;
		limbs_setPositon_legJointSingle(legNum,&jointTemp,angle);
	}
}

void limbs_setPositon_legJointSingle(LegNumTypeDef *legNum, JointNumTypeDef *jointNum, int16_t *angle){
	LegJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, legNum, jointNum) != LimbOK)
		return;

	//set correct direction (multiple by 1 or -1)
	*angle *= joint->servoDirection;

	servo_set_position(joint->servoNum, (type_angle)*angle);
}

LimbStatusTypeDef limbs_getStatus(LegNumTypeDef legNum, JointNumTypeDef jointNum){
	LegJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, &legNum, &jointNum) != LimbOK)
		return LimbError;

	return servo_get_status(joint->servoNum) != Servo_OK ? LimbBusy : LimbOK;
}

void limbs_changeSpeed(LegNumTypeDef legNum, JointNumTypeDef jointNum, LimbSpeedTypeDef speed){
	switch(speed){
	case LimbSpeedVeryFast:	limbs_changeSpeedPercentage(legNum, jointNum,100,100); return;
	case LimbSpeedFast:		limbs_changeSpeedPercentage(legNum, jointNum,60,60); return;
	case LimbSpeedNormal:	limbs_changeSpeedPercentage(legNum, jointNum,40,40); return;
	case LimbSpeedSlow:		limbs_changeSpeedPercentage(legNum, jointNum,25,25); return;
	case LimbSpeedVerySlow:	limbs_changeSpeedPercentage(legNum, jointNum,10,10); return;
	}
}

void limbs_changeSpeedPercentage(LegNumTypeDef legNum, JointNumTypeDef jointNum, int8_t PercentageOfAlphaMax, int8_t PercentageOfOmegaMax){
	if(PercentageOfAlphaMax > 100 || PercentageOfOmegaMax > 100)
		return;

	type_alpha alpha_max;
	type_omega omega_max;

	switch(PercentageOfAlphaMax){
	case PARAMETER_SET_DEFAULT: alpha_max = def_alpha_max;							break;
	case PARAMETER_DONT_CHANGE: alpha_max = servo_DoNotChange; 						break;
	default: 					alpha_max = PercentageOfAlphaMax * def_alpha_max;	break;
	}
	switch(PercentageOfOmegaMax){
	case PARAMETER_SET_DEFAULT: omega_max = def_omega_max;							break;
	case PARAMETER_DONT_CHANGE: omega_max = servo_DoNotChange; 						break;
	default: 					omega_max = PercentageOfOmegaMax * def_omega_max;	break;
	}

	if( (alpha_max < 1 && alpha_max != servo_DoNotChange) || \
		(omega_max < 1 && omega_max != servo_DoNotChange))
		return;
	limbs_setServoParameters(&legNum, &jointNum, &alpha_max, &omega_max);
}

void limbs_setServoParameters(LegNumTypeDef *legNum, JointNumTypeDef *jointNum, type_alpha *alpha_max, type_omega *omega_max){
	LegJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, legNum,jointNum) != LimbOK)
		return;

	servo_set_calc_param(joint->servoNum, servo_DoNotChange , *alpha_max, *omega_max);
}

LimbStatusTypeDef limbs_getJoint(LegJointypeDef **joint, LegNumTypeDef *legNum, JointNumTypeDef *jointNum){
	LegTypeDef *leg;
	if(limbs_getLeg(&leg, legNum) != LimbOK)
		return LimbError;
	return limbs_getJointFromLeg(joint, leg, jointNum);
}

LimbStatusTypeDef limbs_getJointFromLeg(LegJointypeDef **joint, LegTypeDef *leg, JointNumTypeDef *jointNum){
	if(leg == 0)
		return LegError;

	switch (*jointNum){
	case JointAnkle:	*joint =  &(leg->ankle);return LimbOK;
	case JointHip:		*joint =  &(leg->hip);	return LimbOK;
	case JointError:	*joint =  0;			return LimbError;
	}
	*joint =  0;
	return LimbError;
}

LimbStatusTypeDef limbs_getLeg(LegTypeDef **leg, LegNumTypeDef *legNum){
	switch (*legNum){
	case LegLeft:	*leg = &(Legs.left);	return LimbOK;
	case LegRight:	*leg = &(Legs.right);	return LimbOK;
	case LegError:  *leg = 0;				return LimbError;
	}
	*leg = 0;
	return LimbError;
}
