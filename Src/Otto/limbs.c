/*
 * limbs.c
 *
 *  Created on: Jun 12, 2019
 *      Author: Jarosław Królik
 */
#include "limbs.h"
#include "servo.h"

void limbs_init_value(LimbsJointypeDef *joint, uint8_t sevoNum, int8_t servoDirection);

void limbs_setPositon_legSingle(LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum, int16_t *angle);
void limbs_setPositon_legJointSingle(LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum, int16_t *angle);

void limbs_setServoParameters(LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum, type_alpha *alpha_max, type_omega *omega_max);

LimbStatusTypeDef limbs_getJoint(LimbsJointypeDef **joint, LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum);
LimbStatusTypeDef limbs_getJointFromLimbs(LimbsJointypeDef **joint, LimbsLegTypeDef *leg, LimbJointNumTypeDef *jointNum);
LimbStatusTypeDef limbs_getLeg(LimbsLegTypeDef **leg, LimbsNumTypeDef *legNum);

LimbsTypeDef Limbs;

void limbs_init(){
	servo_init();

	limbs_init_value(&(Limbs.left.ankle),	LEG_SERVO_NUM_LEFT_ANKLE,	LEG_SERVO_DIRECTION_LEFT_ANKLE);
	limbs_init_value(&(Limbs.left.hip),		LEG_SERVO_NUM_LEFT_HIP,		LEG_SERVO_DIRECTION_LEFT_HIP);
	limbs_init_value(&(Limbs.right.ankle),	LEG_SERVO_NUM_RIGHT_ANKLE,	LEG_SERVO_DIRECTION_RIGHT_ANKLE);
	limbs_init_value(&(Limbs.right.hip),	LEG_SERVO_NUM_RIGHT_HIP,	LEG_SERVO_DIRECTION_RIGHT_HIP);

	limbs_changeSpeed(LegLeft,	JointAnkle,	LimbSpeedVeryFast);
	limbs_changeSpeed(LegLeft,	JointHip,	LimbSpeedVeryFast);
	limbs_changeSpeed(LegRight,	JointAnkle,	LimbSpeedVeryFast);
	limbs_changeSpeed(LegRight,	JointHip,	LimbSpeedVeryFast);

	limbs_setPositon(LegLeft|LegRight, JointAnkle|JointHip, 0);
}

void limbs_init_value(LimbsJointypeDef *joint, uint8_t sevoNum, int8_t servoDirection){
	joint->servoNum = sevoNum;
	joint->servoDirection = servoDirection;
}

void limbs_setPositon(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, int16_t angle){
	LimbsNumTypeDef legTemp;

	if(legNum & LegLeft){
		legTemp = LegLeft;
		limbs_setPositon_legSingle(&legTemp,&jointNum,&angle);
	}
	if(legNum & LegRight){
		legTemp = LegRight;
		limbs_setPositon_legSingle(&legTemp,&jointNum,&angle);
	}
}

void limbs_setPositonSingle(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, int16_t angle){
	limbs_setPositon_legJointSingle(&legNum, &jointNum,&angle);
}

void limbs_setPositon_legSingle(LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum, int16_t *angle){
	LimbJointNumTypeDef jointTemp;

	if(*jointNum & JointAnkle){
		jointTemp = JointAnkle;
		limbs_setPositon_legJointSingle(legNum,&jointTemp,angle);
	}
	if(*jointNum & JointHip){
		jointTemp = JointHip;
		limbs_setPositon_legJointSingle(legNum,&jointTemp,angle);
	}
}

void limbs_setPositon_legJointSingle(LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum, int16_t *angle){
	LimbsJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, legNum, jointNum) != LimbOK)
		return;

	//set correct direction (multiple by 1 or -1)
	*angle *= joint->servoDirection;

	servo_set_position(joint->servoNum, (type_angle)*angle);
}

LimbStatusTypeDef limbs_getStatus(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum){
	LimbsJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, &legNum, &jointNum) != LimbOK)
		return LimbError;

	return servo_get_status(joint->servoNum) != Servo_OK ? LimbBusy : LimbOK;
}

void limbs_changeSpeed(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, LimbSpeedTypeDef speed){
	switch(speed){
	case LimbSpeedVeryFast:	limbs_changeSpeedPercentage(legNum, jointNum,100,100); return;
	case LimbSpeedFast:		limbs_changeSpeedPercentage(legNum, jointNum,60,60); return;
	case LimbSpeedNormal:	limbs_changeSpeedPercentage(legNum, jointNum,40,40); return;
	case LimbSpeedSlow:		limbs_changeSpeedPercentage(legNum, jointNum,25,25); return;
	case LimbSpeedVerySlow:	limbs_changeSpeedPercentage(legNum, jointNum,10,10); return;
	}
}

void limbs_changeSpeedPercentage(LimbsNumTypeDef legNum, LimbJointNumTypeDef jointNum, int8_t PercentageOfAlphaMax, int8_t PercentageOfOmegaMax){
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

void limbs_setServoParameters(LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum, type_alpha *alpha_max, type_omega *omega_max){
	LimbsJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, legNum,jointNum) != LimbOK)
		return;

	servo_set_calc_param(joint->servoNum, servo_DoNotChange , *alpha_max, *omega_max);
}

LimbStatusTypeDef limbs_getJoint(LimbsJointypeDef **joint, LimbsNumTypeDef *legNum, LimbJointNumTypeDef *jointNum){
	LimbsLegTypeDef *leg;
	if(limbs_getLeg(&leg, legNum) != LimbOK)
		return LimbError;
	return limbs_getJointFromLimbs(joint, leg, jointNum);
}

LimbStatusTypeDef limbs_getJointFromLimbs(LimbsJointypeDef **joint, LimbsLegTypeDef *leg, LimbJointNumTypeDef *jointNum){
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

LimbStatusTypeDef limbs_getLeg(LimbsLegTypeDef **leg, LimbsNumTypeDef *legNum){
	switch (*legNum){
	case LegLeft:	*leg = &(Limbs.left);	return LimbOK;
	case LegRight:	*leg = &(Limbs.right);	return LimbOK;
	case LegError:  *leg = 0;				return LimbError;
	}
	*leg = 0;
	return LimbError;
}
