/*
 * board_data.hpp
 *
 *  Created on: 2023/04/03
 *      Author: yaa3k
 */

#ifndef DRIVER_BOARD_DATA_HPP_
#define DRIVER_BOARD_DATA_HPP_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "../inc/main.h"

#define TIM7_INT
#define TIM7_FRQ 10000
//#define SPI_DMA

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

	adc1_n = 3,
	adc2_n = 4
};

enum class PHASE{
	U,
	V,
	W
};

enum class ENC_type{
	AS5600,
	AS5048,
	ABX,
	UVW_HALL,
	AB_LINER_HALL,
};

#endif /* DRIVER_BOARD_DATA_HPP_ */
