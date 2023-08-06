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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../inc/main.h"

#define TIM7_INT
#define TIM7_FRQ 10000

enum class borard_reg{
	CAN_ID,             //r (set via UART)
	MOTOR_TYPE,         //r/w
	CONTROL_MODE,       //r/w
	ENC_TYPE,           //r/w
	FOC_GAIN_P,         //r/w
	FOC_GAIN_I,         //r/w
	SPD_GAIN_P,         //r/w
	SPD_GAIN_I,         //r/w
	SPD_GAIN_D,         //r/w
	M_POS,              //r
	I_LIMIT,            //r/w
	BATT_VOL            //r
};

enum class motor_type{
	DC,
	FOC,
	BLDC_FORCED_COMM,
	FOC_SENSORLESS,
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
