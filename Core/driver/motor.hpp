/*
 * motor.hpp
 *
 *  Created on: Jun 27, 2023
 *      Author: yaa3k
 */

#ifndef DRIVER_MOTOR_HPP_
#define DRIVER_MOTOR_HPP_

#include "board_data.hpp"
#include "driver.hpp"
#include "pwm.hpp"
#include "motor_math.hpp"
#include "analog_sens.hpp"
#include "encoder.hpp"

class MOTOR : motor_math{
private:
	DRIVER &driver;
	ANALOG_SENS &analog;
	ENCODER &enc;

	//motor electrical angle
	uint16_t angle_e;

	//motor parameter
	float motor_R;
	float motor_L;
	float motor_Ke;

	//FOC control//
	//dq current PID
	PID pid_d;
	PID pid_q;

	//current data
	uvw_t i_uvw;
	dq_t i_dq;
	uvw_t pwm_uvw;
	dq_t v_dq;
	dq_t pwm_dq;

	//target
	dq_t i_dq_target;

	//speed control//
	//speed PID
	PID pid_speed;
	//current data
	float motor_speed_rad;
	//target
	float motor_speed_target;
	LPF<int> speed_filter;

	//for measure speed
	int32_t top = 0;
	int32_t angle_log[8] = {0};
	int32_t angle_diff = 0;

public:
	MOTOR(DRIVER &_driver,ANALOG_SENS &_analog,ENCODER &_enc)
		:driver(_driver),analog(_analog),enc(_enc),
		 pid_d(false,TIM7_FRQ),pid_q(false,TIM7_FRQ),
		 pid_speed(false,TIM7_FRQ),speed_filter(10){
	}
	void init(void);

	void print_debug(void);
	void control(void);

	//measure motor parameter
	void enc_calibration(float duty);
	float measure_R(float duty);
	float measure_L(float R,float duty);
	float measure_speed_rad(float freq);

	//inline functions
	void set_dq_current(dq_t target){
		i_dq_target = target;
	}
	void set_kv(float kv){
		motor_Ke = 60/(2*M_PI*kv);
	}
};



#endif /* DRIVER_MOTOR_HPP_ */
