/*
 * driver.cpp
 *
 *  Created on: Mar 18, 2023
 *      Author: yaa3k
 */

#include "pwm.hpp"

#ifdef PWM_LL
void PWM::out(float val){
	if(val < min || max < val)val  = 0;

	if(polarity_inv) val *= -1;

	set_compare((val - min)*diff_inv*tim_period);
}

uint32_t PWM::get_compare(void){
	switch(ch){
	case LL_TIM_CHANNEL_CH1:
		return LL_TIM_OC_GetCompareCH1(tim);
		break;
	case LL_TIM_CHANNEL_CH2:
		return LL_TIM_OC_GetCompareCH2(tim);
		break;
	case LL_TIM_CHANNEL_CH3:
		return LL_TIM_OC_GetCompareCH2(tim);
		break;
	}
	return 0;
}

void PWM::set_compare(uint32_t comp_val){
	switch(ch){
	case LL_TIM_CHANNEL_CH1:
		LL_TIM_OC_SetCompareCH1(tim,comp_val);
		break;
	case LL_TIM_CHANNEL_CH2:
		LL_TIM_OC_SetCompareCH2(tim,comp_val);
		break;
	case LL_TIM_CHANNEL_CH3:
		LL_TIM_OC_SetCompareCH3(tim,comp_val);
		break;
	}
}
#else
void PWM::out(float val){
	if(val < min || max < val)val  = 0;

	if(polarity_inv) val *= -1;

	__HAL_TIM_SET_COMPARE(tim, ch, (val - min)*diff_inv*tim_period);
}
#endif
