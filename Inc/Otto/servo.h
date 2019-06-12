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
#define def_omega_max 1000 //5,4 degrees per time_base (20ms)
#define def_alpha_max 100 //1,0 degrees per timbe_base squared (20_ms)^2

#define servo_0_range_pos_min 625
#define servo_0_range_pos_mid 1500
#define servo_0_range_pos_max 2500
#define servo_1_range_pos_min 625
#define servo_1_range_pos_mid 1500
#define servo_1_range_pos_max 2500
#define servo_2_range_pos_min 625
#define servo_2_range_pos_mid 1500
#define servo_2_range_pos_max 2500
#define servo_3_range_pos_min 625
#define servo_3_range_pos_mid 1500
#define servo_3_range_pos_max 2500

#define type_n uint32_t
#define type_phi uint32_t
#define type_theta int32_t
#define type_omega int32_t
#define type_alpha int32_t

typedef struct{
	 _Bool in_motion;
	type_n n_current;
	type_omega omega_enh;
	uint32_t *timer_set_position;
}Motion_prm_it;

typedef struct{
	type_n n_max1;
	type_n n_max2;
	type_n n_max3;
	type_theta alpha_enh;
}Motion_calcuated_prm;

typedef struct{
	type_n n_min;
	type_omega omega_max_enh;
	type_alpha alpha_max_enh;
}Motion_calculation_prm;

typedef struct {
	type_phi phi_min_enh;
	type_phi phi_mid_enh;
	type_phi phi_max_enh;
}Position_range;

typedef struct{
	//parameters changinge when motion
	volatile Motion_prm_it prm_it;
	//parameters of calculation
	Motion_calculation_prm calculation_prm;
	//calculated parameters before motion
	Motion_calcuated_prm calcuated_prm;
	//const parameters
	Position_range range;
}Servo;

/*
 * Servo initialization
 */
void servo_init();

/*
 * Servo timer handler
 */
void servo_timer_step_handler();
/*
 * If 0 -> set default parameter
 */
void servo_set_calc_param(uint8_t servo_number, type_n n_min, type_alpha alpha_max, type_omega omega_max);

int8_t servo_set_position(uint8_t servo_number, type_phi phi_set);

void servo_set_position_direct(uint8_t servo_number, type_phi positon);

//for debug
Servo* get_servo(uint8_t number);
void set_motion_prm_n(Servo *servo, type_theta theta_eth, type_n n_acc, type_n n_const, _Bool change_theta_sign);

#endif /* SERVO_H_ */