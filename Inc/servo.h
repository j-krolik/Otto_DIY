/*
 * servo.h
 *
 *  Created on: May 15, 2019
 *      Author: Jarosław Królik
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "stdint.h" //uint32_t

/* descryption
 *
 * phi		- angle * 10					- degree
 * theta	- angular displacement (*10)	- degree
 * omega	- angular speed (*10)			- degree per second
 * alpha	- angular acceleration (*10)	- degree per second squared
 *
 * _enh		- enhanced - value multiplied by accuracy_multiplier
 *
 * n_acc	- alpha != 0
 * n_const	- alpha == 0
 */

#define accuracy_multiplier 10000

#define number_of_servos 4
#define def_n_min 1
#define def_omega_max 54 //5,4 degrees per time_base (20ms)
#define def_alpha_max 10 //1,0 degrees per timbe_base squared (20_ms)^2

#define type_n uint32_t
#define type_phi uint32_t
#define type_theta int32_t
#define type_omega int32_t
#define type_alpha int32_t

typedef struct {
	type_phi phi_min_enh;
	type_phi phi_mid_enh;
	type_phi phi_max_enh;
}Position_range;

typedef struct{
	volatile _Bool in_motion;
	//volatile type_phi phi_current_enh;
	//calculated parameters before motion
	type_n n_max1;
	type_n n_max2;
	type_n n_max3;
	type_theta alpha_enh;
	//parameters calculating when motion
	volatile type_n n_current;
	volatile type_omega omega_enh;
}Motion_prm;

typedef struct{
	Position_range range;
	Motion_prm prm;
	volatile uint32_t *timer_set_position;
}Servo;

typedef struct{
	type_n n_min;
	type_omega omega_max_enh;
	type_alpha alpha_max_enh;
}Motion_max;


void Servo_Init();
void Servo_next_step();
int8_t cal_moution_parameters(uint8_t servo_number, type_phi phi_set_enh);
void set_servo_position(uint8_t servo_number, type_phi positon);

//for debug
Servo* get_servo(uint8_t number);
void set_motion_prm_n(Motion_prm *_motion_prm, type_theta theta_eth, type_n n_acc, type_n n_const, _Bool change_theta_sign);

#endif /* SERVO_H_ */
