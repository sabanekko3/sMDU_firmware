/*
 * table.hpp
 *
 *  Created on: Mar 18, 2023
 *      Author: yaa3k
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include "board_data.hpp"

class PWM{
private:
	TIM_HandleTypeDef *tim;
	const uint32_t ch;
	const uint32_t tim_period;
	const float min;
	const float max;
	const float margin;
	const bool polarity_inv;
	float diff_inv;
	uint32_t pwm_val;

public:
	PWM(TIM_HandleTypeDef *_tim,uint32_t _ch,uint32_t _tim_period,float _min,float _max, float _margin,bool _polarity_inv)
		: tim(_tim),ch(_ch),tim_period(_tim_period),min(_min),max(_max),margin(_margin),polarity_inv(_polarity_inv){
		diff_inv = 1/(max - min);
	}

	void out(float val);//-1~1

	uint32_t get_compare_val(void){
		return pwm_val;
	}

	void start(void);
	void stop(void);
};

#endif


