/*
 * adc.cpp
 *
 *  Created on: Apr 7, 2023
 *      Author: yaa3k
 */
#include "adc.hpp"

void ADC_DMA::start(uint16_t *data){
	//LL_DMA_EnableIT_TC(dma, dma_ch);
	LL_ADC_Enable(adc);

	LL_DMA_DisableChannel(dma,dma_ch);

	LL_DMA_ConfigAddresses(dma, dma_ch,
		                 LL_ADC_DMA_GetRegAddr(adc, LL_ADC_DMA_REG_REGULAR_DATA),
		                 (uint32_t)&data, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);


	LL_DMA_SetDataLength(dma, dma_ch, data_n);

	LL_DMA_EnableChannel(dma,dma_ch);

	LL_ADC_REG_StartConversion(adc);
}
void ADC_DMA::stop(void){
	LL_DMA_DisableChannel(dma,dma_ch);
}
