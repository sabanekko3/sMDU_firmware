/*
 * driver.hpp
 *
 *  Created on: Mar 27, 2023
 *      Author: yaa3k
 */

#ifndef INC_DRIVER_HPP_
#define INC_DRIVER_HPP_

#include "board_data.hpp"
#include "pwm.hpp"
#include "motor_math.hpp"

class DRIVER{
private:
	motor_math& math;

	PWM& pwm_u;
	PWM& pwm_v;
	PWM& pwm_w;

	GPIO_TypeDef *en_port;
	uint16_t en_pin;

public:
	DRIVER(PWM &_pwm_u,PWM &_pwm_v,PWM &_pwm_w,
			GPIO_TypeDef *_en_port,uint16_t _en_pin, motor_math &_math)
		: pwm_u(_pwm_u),pwm_v(_pwm_v),pwm_w(_pwm_w),
		  	  en_port(_en_port),en_pin(_en_pin),math(_math){}

	void out(int angle,float power);
	void out(uvw_t uvw);
	void get_pwm_val(uint32_t *u_val,uint32_t *v_val,uint32_t *w_val){
		*u_val = pwm_u.get_compare_val();
		*v_val = pwm_v.get_compare_val();
		*w_val = pwm_w.get_compare_val();
	}

	void pwms_start(void);
	void pwms_stop(void);
};



#endif /* INC_DRIVER_HPP_ */
