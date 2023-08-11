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

	uint16_t angle_e;

	//dq current PID
	PID pid_d;
	PID pid_q;

	//current data
	uvw_t i_uvw;
	dq_t i_dq;
	uvw_t v_uvw;
	dq_t v_dq;

	//target_data
	dq_t i_dq_target;

public:
	MOTOR(DRIVER &_driver,ADC &_adc,ENCODER &_enc)
		:driver(_driver),adc(_adc),enc(_enc),
		 pid_d(false,TIM7_FRQ),pid_q(false,TIM7_FRQ){

	}
	void init(void);

	void print_debug(void);
	void control(void);

	//inline functions
	void set_dq_current(dq_t target){
		i_dq_target = target;
	}
};



#endif /* DRIVER_MOTOR_HPP_ */
