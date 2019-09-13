/*
 * servo.c
 *
 *  Created on: May 15, 2019
 *      Author: Jarosław Królik
 */

#include <Otto/servo.h>
#include "stdbool.h" //_Bool
#include "math.h" //sqrt
#include "tim.h"

/***** functions declaration *****/
/*** set new position ***/
int8_t servo_set_position_phi(uint8_t servo_number, type_phi phi_set);
void set_motion_prm_n(Servo *servo, type_theta theta_eth, type_n n_acc, type_n n_const, _Bool change_theta_sign);
inline type_phi get_servo_position_enh(Servo *_servo);

inline void servo_set_position_direct(uint8_t servo_number, type_phi positon);

/***** macros *****/
#define __servo_range_pos(number,param) servo_ ## number ## _range_pos_ ## param
#define square(_param_) (_param_*_param_)

/***** private variable *****/
Motion_calculation_prm default_calculation_prm;
Servo servo[number_of_servos];

/***** functions  *****/
void servo_init(){
	default_calculation_prm.n_min = def_n_min;
	default_calculation_prm.omega_max_enh = def_omega_max * accuracy_multiplier;
	default_calculation_prm.alpha_max_enh = def_alpha_max * accuracy_multiplier;

	//define PWM timer adress
	servo[0].prm_it.timer_set_position = (uint32_t*)&(OTTO_TIM_servo->CCR1);
	servo[1].prm_it.timer_set_position = (uint32_t*)&(OTTO_TIM_servo->CCR2);
	servo[2].prm_it.timer_set_position = (uint32_t*)&(OTTO_TIM_servo->CCR3);
	servo[3].prm_it.timer_set_position = (uint32_t*)&(OTTO_TIM_servo->CCR4);

	//define servo ranges
	servo[0].range.phi_min_enh = __servo_range_pos(0,min) * accuracy_multiplier;
	servo[0].range.phi_mid_enh = __servo_range_pos(0,mid) * accuracy_multiplier;
	servo[0].range.phi_max_enh = __servo_range_pos(0,max) * accuracy_multiplier;
	servo[1].range.phi_min_enh = __servo_range_pos(1,min) * accuracy_multiplier;
	servo[1].range.phi_mid_enh = __servo_range_pos(1,mid) * accuracy_multiplier;
	servo[1].range.phi_max_enh = __servo_range_pos(1,max) * accuracy_multiplier;
	servo[2].range.phi_min_enh = __servo_range_pos(2,min) * accuracy_multiplier;
	servo[2].range.phi_mid_enh = __servo_range_pos(2,mid) * accuracy_multiplier;
	servo[2].range.phi_max_enh = __servo_range_pos(2,max) * accuracy_multiplier;
	servo[3].range.phi_min_enh = __servo_range_pos(3,min) * accuracy_multiplier;
	servo[3].range.phi_mid_enh = __servo_range_pos(3,mid) * accuracy_multiplier;
	servo[3].range.phi_max_enh = __servo_range_pos(3,max) * accuracy_multiplier;

	for(uint8_t i=0; i<number_of_servos; i++){
		servo[i].prm_it.in_motion = false;
		servo_set_calc_param(i,0,0,0);
		servo_set_position_direct(i, servo[i].range.phi_mid_enh/accuracy_multiplier);
	}
}

void servo_step_timer_handler(volatile uint8_t *free_servos){
	*free_servos = 0;
	for(uint8_t i = 0; i<number_of_servos ;i++){
		//chceck if servo sholud move
		if(servo[i].prm_it.in_motion != false){
			Motion_prm_it *prm_it = (Motion_prm_it*)&(servo[i].prm_it); //i konw -> full optimisation, no volatile
			Motion_calcuated_prm *calcuated_prm = &(servo[i].calcuated_prm);

			//calculate current omega
			if(prm_it->n_current < calcuated_prm->n_max1)
				prm_it->omega_enh += calcuated_prm->alpha_enh;
			else if(prm_it->n_current < calcuated_prm->n_max2);
			else if(prm_it->n_current < calcuated_prm->n_max3 && prm_it->omega_enh!=0)
				prm_it->omega_enh -= calcuated_prm->alpha_enh;
			else{
				prm_it->in_motion = false;
				*free_servos |= (1<<i);
				break;
			}
			//calculate current position
			*(prm_it->timer_set_position) += prm_it->omega_enh / accuracy_multiplier;
			prm_it->n_current += 1;
		}else
			//set bit - servo is free
			*free_servos |= (1<<i);
	}
}

void servo_set_calc_param(uint8_t servo_number, type_n n_min, type_alpha alpha_max, type_omega omega_max){
	if(n_min != servo_DoNotChange)
		servo[servo_number].calculation_prm.n_min = \
			(n_min == servo_SetDefault || n_min < default_calculation_prm.n_min) ? default_calculation_prm.n_min : n_min;

	if(alpha_max != servo_DoNotChange)
		servo[servo_number].calculation_prm.alpha_max_enh = \
			(alpha_max == servo_SetDefault || alpha_max*accuracy_multiplier > default_calculation_prm.alpha_max_enh) ? \
			default_calculation_prm.alpha_max_enh : alpha_max*accuracy_multiplier;

	if(omega_max != servo_DoNotChange)
		servo[servo_number].calculation_prm.omega_max_enh = \
			(omega_max == servo_SetDefault || omega_max*accuracy_multiplier > default_calculation_prm.omega_max_enh) ? \
			default_calculation_prm.omega_max_enh	: omega_max*accuracy_multiplier;
}

int8_t servo_set_position(uint8_t servo_number, type_angle angle_set){
	type_phi positon;
	type_phi mid = servo[servo_number].range.phi_mid_enh;
	if(angle_set == 0)
		positon = mid;
	else if(angle_set > 0){
		type_phi max = servo[servo_number].range.phi_max_enh;
		positon = mid + ((max-mid) * (type_phi)(angle_set)) / 90;
	}else{
		type_phi min = servo[servo_number].range.phi_min_enh;
		positon = mid - ((mid-min) * (type_phi)(-1*angle_set)) / 90;
	}

	return servo_set_position_phi(servo_number, positon);
}

int8_t servo_set_position_phi(uint8_t servo_number, type_phi phi_set_enh){
	//chceck if servo in motion
	if(servo[servo_number].prm_it.in_motion == true)
		return -1;

	//Motion_prm *_motion_prm = &(servo[servo_number].prm);
	Position_range *_position_range = &(servo[servo_number].range);
	type_phi phi_current_enh = get_servo_position_enh(&servo[servo_number]);

	//check if phi_set is in range
	//type_phi phi_set_enh = phi_set * accuracy_multiplier;
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
	Motion_calculation_prm *servo_calculation_prm = &(servo[servo_number].calculation_prm);
	type_n n_omega_max = servo_calculation_prm->omega_max_enh / servo_calculation_prm->alpha_max_enh;
	type_alpha alpha_max_calc_enh;
	//check if acceleration too high
	if(n_omega_max < servo_calculation_prm->n_min){
		n_omega_max = servo_calculation_prm->n_min;
		//decrease acceleration
		alpha_max_calc_enh = servo_calculation_prm->omega_max_enh / n_omega_max;
	}else
		alpha_max_calc_enh = servo_calculation_prm->alpha_max_enh;
	type_omega omega_max_cal_enh = n_omega_max * alpha_max_calc_enh;


	//determine min steps
	type_n n_cal =  (type_n)sqrt(theta_enh / alpha_max_calc_enh);
	if(n_cal < servo_calculation_prm->n_min){
		//motion type 1
		// the acceleration is too high
		type_alpha alpha_final_eth = theta_enh/square(servo_calculation_prm->n_min);
		set_motion_prm_n(&(servo[servo_number]), alpha_final_eth, servo_calculation_prm->n_min, 0, negative_sign_correction);
		return 1;
	}else if(n_cal<n_omega_max){
		//motion type 2
		//n_cal was round down -> epsilon is inversely proportional to square of n
		n_cal += 1;
		//calc alpha
		type_alpha alpha_final_eth = theta_enh/square(n_cal);
		//for integers only
		if(alpha_final_eth == 0){
			alpha_final_eth = 1;
			n_cal = (type_n)sqrt(theta_enh / alpha_final_eth);
		}
		set_motion_prm_n(&(servo[servo_number]), alpha_final_eth, n_cal, 0, negative_sign_correction);
		return 2;
	}else{
		//motion type 3
		type_n n_acc = n_omega_max;
		type_theta theta_1_3_phase = alpha_max_calc_enh * (n_acc*n_acc);
		type_theta theta_2_phase = theta_enh - theta_1_3_phase;

		type_n n_const = (theta_2_phase / omega_max_cal_enh) + 1; // +1 -> reduce alpha max to max value

		type_alpha alpha_final_eth = theta_enh / (n_acc*(n_acc + n_const));
		set_motion_prm_n(&(servo[servo_number]), alpha_final_eth, n_acc, n_const, negative_sign_correction);
		return 3;
	}

}

type_phi get_servo_position_enh(Servo *_servo){
	return (type_phi)*(_servo->prm_it.timer_set_position) * accuracy_multiplier;
}

void set_motion_prm_n(Servo *servo, type_alpha alpha_enh, type_n n_acc, type_n n_const, _Bool change_theta_sign){
	if(servo->prm_it.in_motion != false)
		return;
	if(change_theta_sign)
		servo->calcuated_prm.alpha_enh = -alpha_enh;
	else
		servo->calcuated_prm.alpha_enh = alpha_enh;
	servo->calcuated_prm.n_max1 = n_acc;
	servo->calcuated_prm.n_max2 = servo->calcuated_prm.n_max1 + n_const;
	servo->calcuated_prm.n_max3 = servo->calcuated_prm.n_max2 + n_acc;
	servo->prm_it.n_current = 0;
	servo->prm_it.omega_enh = 0;
	servo->prm_it.in_motion = true;
}

ServoStatus servo_get_status(uint8_t servo_number){
	return servo[servo_number].prm_it.in_motion ? Servo_Busy : Servo_OK;
}

void servo_set_position_direct(uint8_t servo_number, type_phi positon){
	*(servo[servo_number].prm_it.timer_set_position) = positon;
}










