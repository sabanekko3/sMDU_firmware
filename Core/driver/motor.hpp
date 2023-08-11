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

class MOTOR{
private:
	DRIVER &driver;
	motor_math &math;
	ADC &adc;
	ENCODER &enc;

	uint16_t angle_e;
	uint16_t angle_m;

	//current data
	uvw_t i_uvw;
	dq_t i_dq = {0,0};
	uvw_t v_uvw;
	dq_t v_dq;

	//target_data
	dq_t i_dq_target;

public:
	MOTOR(DRIVER &_driver,ADC &_adc,motor_math &_math,ENCODER &_enc)
		:driver(_driver),adc(_adc),math(_math),enc(_enc){}
	void init(void);

	void print_debug(void);
	void control(void);

	void set_dq_current(dq_t target){
		i_dq_target = target;
	}
};



#endif /* DRIVER_MOTOR_HPP_ */
