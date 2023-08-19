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

class ADC_DMA{
private:
	static constexpr uint32_t DATA_MAX = 8;
	ADC_TypeDef *adc;
	DMA_TypeDef *dma;
	const uint32_t dma_ch;
	const uint32_t data_n;
	uint16_t *data;

public:
	ADC_DMA(ADC_TypeDef *_adc,DMA_TypeDef *_dma,uint32_t _dma_ch,uint32_t _data_n):
		adc(_adc),dma(_dma),dma_ch(_dma_ch),data_n(_data_n){

		for(int i=0; i<DATA_MAX;i++){
			data[i]=0;
		}
	}

	void start(uint16_t *data);
	void stop(void);


};



#endif /* DRIVER_ADC_HPP_ */
