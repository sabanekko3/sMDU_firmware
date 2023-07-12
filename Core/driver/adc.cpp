/*
 * adc.cpp
 *
 *  Created on: Apr 7, 2023
 *      Author: yaa3k
 */
#include "adc.hpp"

void ADC::init(void){
	dma_start();
	HAL_Delay(100);
	for(uint8_t i = 0; i < 16; i++){
		for(uint8_t j = 0; j < (int)ADC_data::n; j++){
			adc_init[j] += adc_dma[j];
		}
		HAL_Delay(1);
	}
	for(uint8_t i = 0; i < (int)ADC_data::n; i++){
		adc_init[i] /= 16;
	}
}

//#define HPF_ACTIVE
#ifdef HPF_ACTIVE
HPF hpf_u(0.01);
HPF hpf_v(0.01);
HPF hpf_w(0.01);
#endif
uvw_t ADC::get_i_uvw(void){
	uvw_t i;

#ifdef HPF_ACTIVE
	i.u = hpf_u.solve((adc_dma[(int)ADC_data::U_I] - adc_init[(int)ADC_data::U_I]) * i_gain);
	i.v = hpf_v.solve((adc_dma[(int)ADC_data::V_I] - adc_init[(int)ADC_data::V_I]) * i_gain);
	i.w = hpf_w.solve((adc_dma[(int)ADC_data::W_I] - adc_init[(int)ADC_data::W_I]) * i_gain);
#else
	i.u = (adc_dma[(int)ADC_data::U_I] - adc_init[(int)ADC_data::U_I]) * i_gain;
	i.v = (adc_dma[(int)ADC_data::V_I] - adc_init[(int)ADC_data::V_I]) * i_gain;
	i.w = (adc_dma[(int)ADC_data::W_I] - adc_init[(int)ADC_data::W_I]) * i_gain;
	//i.w = -i.v-i.u;
	int data_state = (i.u<0?0:0b100) + (i.v<0?0:0b10) + (i.w<0?0:0b1);
	switch (data_state) {
	case 0b011:
		i.u = -i.v - i.w;
	    break;
	case 0b101:
	    i.v = -i.u - i.w;
	    break;
	case 0b110:
		i.w = -i.u - i.v;
	    break;
	default:
	    break;
	}
#endif
	return i;
}

float ADC::get_power_v(void){
	return (float)adc_dma[(int)ADC_data::POWER_V]*v_gain;
}

void ADC::dma_start(void){
	HAL_ADC_Start_DMA(adc1, (uint32_t*) &adc_dma[(int)ADC_data::W_I], (int)ADC_data::adc1_n);
	HAL_ADC_Start_DMA(adc2, (uint32_t*) &adc_dma[(int)ADC_data::U_I], (int)ADC_data::adc2_n);
}

void ADC::dma_stop(void){
	HAL_ADC_Stop_DMA(adc1);
	HAL_ADC_Stop_DMA(adc2);
}
