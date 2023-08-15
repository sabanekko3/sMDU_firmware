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
#include "adc.hpp"
#include "encoder.hpp"

class MOTOR : motor_math{
private:
	DRIVER &driver;
	ADC &adc;
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
	uvw_t v_uvw;
	dq_t v_dq;

	//target
	dq_t i_dq_target;

	//speed control//
	//speed PID
	PID pid_speed;
	//current data
	float motor_speed;
	//target
	float motor_speed_target;
	LPF speed_filter;

	//for measure speed
	int32_t top = 0;
	float rad_log[8] = {0};
	float rad_diff = 0;

public:
	MOTOR(DRIVER &_driver,ADC &_adc,ENCODER &_enc)
		:driver(_driver),adc(_adc),enc(_enc),
		 pid_d(false,TIM7_FRQ),pid_q(false,TIM7_FRQ),
		 pid_speed(false,TIM7_FRQ),speed_filter(0.5){
	}
	void init(void);

	void print_debug(void);
	void control(void);

	//measure motor parameter
	void enc_calibration(float duty);
	float measure_R(float duty);
	float measure_L(float R,float duty);
	float measure_speed(float freq);

	float get_speed_rad(void);

	//inline functions
	void set_dq_current(dq_t target){
		i_dq_target = target;
	}
	void set_kv(float kv){
		motor_Ke = 60/(2*M_PI*kv);
	}
};



#endif /* DRIVER_MOTOR_HPP_ */
