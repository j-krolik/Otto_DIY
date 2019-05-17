/*
 * servo.c
 *
 *  Created on: May 15, 2019
 *      Author: Jarosław Królik
 */

#include "servo.h"
#include "stdbool.h" //_Bool
#include "math.h" //sqrt
#include "tim.h"


type_phi get_servo_position_enh(Servo *_servo);


Motion_max motion_max;
Servo servo[number_of_servos];

void Servo_Init(){
	motion_max.n_min = def_n_min;
	motion_max.omega_max_enh = def_omega_max * accuracy_multiplier;
	motion_max.alpha_max_enh = def_alpha_max * accuracy_multiplier;

	servo[0].timer_set_position = &(TIM2->CCR1);
	servo[1].timer_set_position = &(TIM2->CCR2);
	servo[2].timer_set_position = &(TIM2->CCR3);
	servo[3].timer_set_position = &(TIM2->CCR4);

	for(uint8_t i=0; i<number_of_servos; i++){
		servo[i].prm.in_motion = false;
		servo[i].range.phi_min_enh = 625 * accuracy_multiplier;
		servo[i].range.phi_mid_enh = 1500 * accuracy_multiplier;
		servo[i].range.phi_max_enh = 2500 * accuracy_multiplier;
		set_servo_position(i, servo[i].range.phi_mid_enh/accuracy_multiplier);

	}
}

int8_t cal_moution_parameters(uint8_t servo_number, type_phi phi_set){
	Motion_max *_motion_max = &motion_max;
	Motion_prm *_motion_prm = &(servo[servo_number].prm);
	Position_range *_position_range = &(servo[servo_number].range);
	type_phi phi_current_enh = get_servo_position_enh(&servo[servo_number]);

	//chceck if servo in motion
	if(_motion_prm->in_motion)
		return -1;

	//check if phi_set is in range
	type_phi phi_set_enh = phi_set * accuracy_multiplier;
	if(phi_set_enh>_position_range->phi_max_enh)
		phi_set_enh = _position_range->phi_max_enh;
	else if(phi_set_enh<_position_range->phi_min_enh)
		phi_set_enh = _position_range->phi_min_enh;
	else if(phi_set_enh == phi_current_enh)
		return 0;

	//transpose to phi: 0->A, A>0
	type_theta theta_enh = (type_theta)phi_set_enh - phi_current_enh;
	_Bool negative_sign_correction = false;
	if(theta_enh < 0){
		theta_enh *= -1;
		negative_sign_correction = true;
	}


	//set true o_max (if n_omega_max is reached, then omega_max is reached)
	type_n n_omega_max = _motion_max->omega_max_enh / _motion_max->alpha_max_enh;
	type_alpha alpha_max_calc_enh;
	//check if acceleration too high
	if(n_omega_max < _motion_max->n_min){
		n_omega_max = _motion_max->n_min;
		//decrease acceleration
		alpha_max_calc_enh = _motion_max->omega_max_enh / n_omega_max;
	}else
		alpha_max_calc_enh = _motion_max->alpha_max_enh;
	type_omega omega_max_cal_enh = n_omega_max * alpha_max_calc_enh;


	//determine min steps
	type_n n_cal =  (type_n)sqrt(theta_enh / alpha_max_calc_enh);
	if(n_cal < _motion_max->n_min){
		//motion type 1
		// the acceleration is too high
		type_alpha alpha_final_eth = theta_enh/(_motion_max->n_min^2);
		set_motion_prm_n(_motion_prm, alpha_final_eth, _motion_max->n_min, 0, negative_sign_correction);
		return 1;
	}else if(n_cal<n_omega_max){
		//motion type 2
		//n_cal was round down -> epsilon is inversely proportional to square of n
		n_cal += 1;
		//calc alpha
		type_alpha alpha_final_eth = theta_enh/(n_cal^2);
		//for integers only
		if(alpha_final_eth == 0){
			alpha_final_eth = 1;
			n_cal = (type_n)sqrt(theta_enh / alpha_final_eth);
		}
		set_motion_prm_n(_motion_prm, alpha_final_eth, n_cal, 0, negative_sign_correction);
		return 2;
	}else{
		//motion type 3
		type_n n_acc = n_omega_max;
		type_theta theta_1_3_phase = alpha_max_calc_enh * (n_acc*n_acc);
		type_theta theta_2_phase = theta_enh - theta_1_3_phase;

		type_n n_const = (theta_2_phase / omega_max_cal_enh) + 1; // +1 -> reduce alpha max to max value

		type_alpha alpha_final_eth = theta_enh / (n_acc*(n_acc + n_const));
		set_motion_prm_n(_motion_prm, alpha_final_eth, n_acc, n_const, negative_sign_correction);
		return 3;
	}

}

void set_motion_prm_n(Motion_prm *_motion_prm, type_alpha alpha_enh, type_n n_acc, type_n n_const, _Bool change_theta_sign){
	if(_motion_prm->in_motion != false)
		return;
	if(change_theta_sign)
		_motion_prm->alpha_enh = -alpha_enh;
	else
		_motion_prm->alpha_enh = alpha_enh;
	_motion_prm->n_max1 = n_acc;
	_motion_prm->n_max2 = _motion_prm->n_max1 + n_const;
	_motion_prm->n_max3 = _motion_prm->n_max2 + n_acc;
	_motion_prm->n_current = 0;
	_motion_prm->omega_enh = 0;
	_motion_prm->in_motion = true;
}

Servo* get_servo(uint8_t number){
	return &servo[number];
}

type_phi get_servo_position_enh(Servo *_servo){
	return (type_phi)*(_servo->timer_set_position) * accuracy_multiplier;
}


void Servo_next_step(){
	for(uint8_t i = 0; i<number_of_servos ;i++){
		//chceck if servo sholud move
		if(servo[i].prm.in_motion != false){
			Motion_prm *prm = &(servo[i].prm);
			if(prm->n_current < prm->n_max1)
				prm->omega_enh += prm->alpha_enh;
			else if(prm->n_current < prm->n_max2);
			else if(prm->n_current < prm->n_max3 && prm->omega_enh!=0)
				prm->omega_enh -= prm->alpha_enh;
			else{
				prm->in_motion = false;
				return;
			}
			*(servo[i].timer_set_position) += prm->omega_enh / accuracy_multiplier;
			prm->n_current += 1;
		}
	}

}


void set_servo_position(uint8_t servo_number, type_phi positon){
	*(servo[servo_number].timer_set_position) = (uint32_t)positon;
}
