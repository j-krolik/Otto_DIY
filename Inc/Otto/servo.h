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

#define servo_0_range_pos_min 580
#define servo_0_range_pos_mid 1430
#define servo_0_range_pos_max 2410
#define servo_1_range_pos_min 630
#define servo_1_range_pos_mid 1500
#define servo_1_range_pos_max 2460
#define servo_2_range_pos_min 620
#define servo_2_range_pos_mid 1430
#define servo_2_range_pos_max 2410
#define servo_3_range_pos_min 640
#define servo_3_range_pos_mid 1480
#define servo_3_range_pos_max 2420

#define type_n		int32_t
#define type_angle	int16_t
#define type_phi	uint32_t
#define type_theta	int32_t
#define type_omega	int32_t
#define type_alpha	int32_t

#define servo_SetDefault  -1
#define servo_DoNotChange  0

typedef enum{
	Servo_OK	= 0x0u,
	Servo_Busy	= 0x1u,
	Servo_Error	= 0x2u
}ServoStatus;

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
	//parameters changing when motion
	volatile Motion_prm_it prm_it;
	//parameters used for calculation
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
 *
 * return information with servos is free
 * (least signified bit is servo number 0, next bit is servo number 1...)
 */
void servo_step_timer_handler(volatile uint8_t *free_servos);

/*
 * servo_SetDefault, servo_DoNotChange - extra parameters
 */
void servo_set_calc_param(uint8_t servo_number, type_n n_min, type_alpha alpha_max, type_omega omega_max);

int8_t servo_set_position(uint8_t servo_number, type_angle angle_set);
ServoStatus servo_get_status(uint8_t servo_number);

#endif /* SERVO_H_ */
