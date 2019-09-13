/*
 * limbs.c
 *
 *  Created on: Jun 12, 2019
 *      Author: Jarosław Królik
 */
#include "limbs.h"
#include "servo.h"

void limbs_initValue(LimbsJointypeDef *joint, uint8_t sevoNum, int8_t servoDirection);

void limbs_changeSpeedPercentage(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum, int8_t PercentageOfAlphaMax, int8_t PercentageOfOmegaMax);
void limbs_changeSpeedSend2Servo(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, type_alpha *alpha_max, type_omega *omega_max);

void limbs_functionLimbsMultiJointMulti(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum,
		void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*), void *param);
void limbs_functionLimbsSingleJointMulti(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, \
		void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*), void *param);
void limbs_functionSetPositon(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, void *angleVoid);
void limbs_functionChangeSpeed(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, void *speedVoid);
void limbs_functionGetStatus(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, void *pStatusVoid);

LimbStatusTypeDef limbs_getJoint(LimbsJointypeDef **joint, LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum);
LimbStatusTypeDef limbs_getJointFromLimbs(LimbsJointypeDef **joint, LimbsLegTypeDef *limb, LimbJointNumTypeDef *jointNum);
LimbStatusTypeDef limbs_getLeg(LimbsLegTypeDef **limb, LimbsNumTypeDef *limbNum);

LimbsTypeDef Limbs;

void limbs_init(){
	servo_init(); //including set initial position (middle)

	//initialize limbs value
	limbs_initValue(&(Limbs.left.ankle),	LEG_SERVO_NUM_LEFT_ANKLE,	LEG_SERVO_DIRECTION_LEFT_ANKLE);
	limbs_initValue(&(Limbs.left.hip),		LEG_SERVO_NUM_LEFT_HIP,		LEG_SERVO_DIRECTION_LEFT_HIP);
	limbs_initValue(&(Limbs.right.ankle),	LEG_SERVO_NUM_RIGHT_ANKLE,	LEG_SERVO_DIRECTION_RIGHT_ANKLE);
	limbs_initValue(&(Limbs.right.hip),	LEG_SERVO_NUM_RIGHT_HIP,	LEG_SERVO_DIRECTION_RIGHT_HIP);

	//initialize speed value
	limbs_changeSpeedMulti(LegLeft|LegRight, JointAnkle|JointHip,	LimbSpeedNormal);
}

void limbs_initValue(LimbsJointypeDef *joint, uint8_t sevoNum, int8_t servoDirection){
	joint->servoNum = sevoNum;
	joint->servoDirection = servoDirection;
}

void limbs_setPositon(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum, int16_t angle){
	LimbsJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, &limbNum, &jointNum) != LimbOK)
		return;

	servo_set_position(joint->servoNum, (type_angle)(angle * joint->servoDirection));
}

void limbs_setPositonMulti(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum, int16_t angle){
	//make pointer of function set position
	void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*) = limbs_functionSetPositon;

	//pass function and angle to function which execute function for all limbs and their joint
	limbs_functionLimbsMultiJointMulti(&limbNum, &jointNum, pfunction, (void*) &angle);
}

void limbs_changeSpeed(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum, LimbSpeedTypeDef speed){
	switch(speed){
	case LimbSpeedVeryFast:	limbs_changeSpeedPercentage(limbNum, jointNum,100,100); return;
	case LimbSpeedFast:		limbs_changeSpeedPercentage(limbNum, jointNum,90,40); return;
	case LimbSpeedNormal:	limbs_changeSpeedPercentage(limbNum, jointNum,50,30); return;
	case LimbSpeedSlow:		limbs_changeSpeedPercentage(limbNum, jointNum,30,10); return;
	case LimbSpeedVerySlow:	limbs_changeSpeedPercentage(limbNum, jointNum,10,10); return;
	}
}

void limbs_changeSpeedMulti(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum, LimbSpeedTypeDef speed){
	//make pointer of function get status
	void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*) = limbs_functionChangeSpeed;

	//pass function and angle to function which execute function for all limbs and their joint
	limbs_functionLimbsMultiJointMulti(&limbNum, &jointNum, pfunction, (void*) &speed);
}

void limbs_changeSpeedPercentage(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum, int8_t PercentageOfAlphaMax, int8_t PercentageOfOmegaMax){
	if(PercentageOfAlphaMax > 100 || PercentageOfOmegaMax > 100)
		return;

	type_alpha alpha_max;
	type_omega omega_max;

	switch(PercentageOfAlphaMax){
	case PARAMETER_SET_DEFAULT: alpha_max = def_alpha_max;								break;
	case PARAMETER_DONT_CHANGE: alpha_max = servo_DoNotChange; 							break;
	default: 					alpha_max = PercentageOfAlphaMax * def_alpha_max / 100;	break;
	}
	switch(PercentageOfOmegaMax){
	case PARAMETER_SET_DEFAULT: omega_max = def_omega_max;								break;
	case PARAMETER_DONT_CHANGE: omega_max = servo_DoNotChange; 							break;
	default: 					omega_max = PercentageOfOmegaMax * def_omega_max / 100;	break;
	}

	if( (alpha_max < 1 && alpha_max != servo_DoNotChange) || \
		(omega_max < 1 && omega_max != servo_DoNotChange))
		return;
	limbs_changeSpeedSend2Servo(&limbNum, &jointNum, &alpha_max, &omega_max);
}

void limbs_changeSpeedSend2Servo(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, type_alpha *alpha_max, type_omega *omega_max){
	LimbsJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, limbNum,jointNum) != LimbOK)
		return;

	servo_set_calc_param(joint->servoNum, servo_DoNotChange , *alpha_max, *omega_max);
}

LimbStatusTypeDef limbs_getStatus(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum){
	LimbsJointypeDef *joint = 0;
	if(limbs_getJoint(&joint, &limbNum, &jointNum) != LimbOK)
		return LimbError;

	return servo_get_status(joint->servoNum) != Servo_OK ? LimbBusy : LimbOK;
}

LimbStatusTypeDef limbs_getStatusMulit(LimbsNumTypeDef limbNum, LimbJointNumTypeDef jointNum){
	//make pointer of function get status
	void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*) = limbs_functionGetStatus;

	LimbStatusTypeDef status = LimbOK;
	//pass function and angle to function which execute function for all limbs and their joint
	limbs_functionLimbsMultiJointMulti(&limbNum, &jointNum, pfunction, (void*) &status);

	if (status & LimbError)
		return LimbError;
	if (status & LimbBusy)
		return LimbBusy;
	return LimbOK;
}

void limbs_functionLimbsMultiJointMulti(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, \
		void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*), void *param){

	LimbsNumTypeDef limbTemp;
	if(*limbNum & LegLeft){
		limbTemp = LegLeft;
		limbs_functionLimbsSingleJointMulti(&limbTemp,jointNum,pfunction,param);
	}
	if(*limbNum & LegRight){
		limbTemp = LegRight;
		limbs_functionLimbsSingleJointMulti(&limbTemp,jointNum,pfunction,param);
	}
}

void limbs_functionLimbsSingleJointMulti(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, \
		void (*pfunction)(LimbsNumTypeDef*, LimbJointNumTypeDef*, void*), void *param){

	LimbJointNumTypeDef jointTemp;
	if(*jointNum & JointAnkle){
		jointTemp = JointAnkle;
		pfunction(limbNum,&jointTemp,param);
	}
	if(*jointNum & JointHip){
		jointTemp = JointHip;
		pfunction(limbNum,&jointTemp,param);
	}
}

void limbs_functionSetPositon(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, void *angleVoid){
	limbs_setPositon(*limbNum, *jointNum, *(int16_t*)angleVoid);
}

void limbs_functionChangeSpeed(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, void *speedVoid){
	limbs_changeSpeed(*limbNum, *jointNum, *(LimbSpeedTypeDef*)speedVoid);
}

void limbs_functionGetStatus(LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum, void *pStatusVoid){
	//chceck if flag is set (LimbBusy)
	LimbStatusTypeDef *pStatus = (LimbStatusTypeDef*)pStatusVoid;
	//if flag is set, don not waste time, skip function
	if( *pStatus & LimbBusy)
		return;

	*pStatus = limbs_getStatus(*limbNum, *jointNum);
}

LimbStatusTypeDef limbs_getJoint(LimbsJointypeDef **joint, LimbsNumTypeDef *limbNum, LimbJointNumTypeDef *jointNum){
	LimbsLegTypeDef *limb;
	if(limbs_getLeg(&limb, limbNum) != LimbOK)
		return LimbError;
	return limbs_getJointFromLimbs(joint, limb, jointNum);
}

LimbStatusTypeDef limbs_getJointFromLimbs(LimbsJointypeDef **joint, LimbsLegTypeDef *limb, LimbJointNumTypeDef *jointNum){
	if(limb == 0)
		return LegError;

	switch (*jointNum){
	case JointAnkle:	*joint =  &(limb->ankle);return LimbOK;
	case JointHip:		*joint =  &(limb->hip);	return LimbOK;
	case JointError:	*joint =  0;			return LimbError;
	}
	*joint =  0;
	return LimbError;
}

LimbStatusTypeDef limbs_getLeg(LimbsLegTypeDef **limb, LimbsNumTypeDef *limbNum){
	switch (*limbNum){
	case LegLeft:	*limb = &(Limbs.left);	return LimbOK;
	case LegRight:	*limb = &(Limbs.right);	return LimbOK;
	case LegError:  *limb = 0;				return LimbError;
	}
	*limb = 0;
	return LimbError;
}
