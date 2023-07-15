/*
 * driver.cpp
 *
 *  Created on: Mar 18, 2023
 *      Author: yaa3k
 */

#include "pwm.hpp"

void PWM::out(float val){
	if(val < min+margin || max-margin < val){
		val  = 0;
	}
	pwm_val = (val - min)*diff_inv*tim_period;

	__HAL_TIM_SET_COMPARE(tim, ch, pwm_val);
}
void PWM::start(void){
	HAL_TIM_PWM_Start(tim, ch);
	__HAL_TIM_SET_COMPARE(tim, ch,0);
}

void PWM::stop(void){
	HAL_TIM_PWM_Stop(tim, ch);
	__HAL_TIM_SET_COMPARE(tim, ch,0);
}
