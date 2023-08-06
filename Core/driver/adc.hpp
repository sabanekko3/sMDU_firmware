/*
 * adc.hpp
 *
 *  Created on: Apr 7, 2023
 *      Author: yaa3k
 */

#ifndef DRIVER_ADC_HPP_
#define DRIVER_ADC_HPP_

#include "board_data.hpp"
#include "motor_math.hpp"

enum class ADC_data{
	//adc1
	W_I,
	RBM_H2,
	//adc2
	U_I,
	V_I,
	POWER_V,
	RBM_H1,
	n,

	adc1_n = 2,
	adc2_n = 4
};

class ADC{
private:
	ADC_HandleTypeDef *adc1;
	ADC_HandleTypeDef *adc2;
	const float i_gain;
	const float v_gain;
	uint16_t adc_init[(int)ADC_data::n] = {0};
	uint16_t adc_dma[(int)ADC_data::n] = {0};
public:
	ADC(ADC_HandleTypeDef *_adc1,ADC_HandleTypeDef *_adc2,float _i_gain,float _v_gain)
		:adc1(_adc1),adc2(_adc2),i_gain(_i_gain),v_gain(_v_gain){}

	void init(void);
	uvw_t get_i_uvw(void);

	float get_power_v(void);

	void dma_start(void);
	void dma_stop(void);

	template<typename T>
	uint16_t get_raw(T d){
		return adc_dma[(int)d];
	}
};



#endif /* DRIVER_ADC_HPP_ */
