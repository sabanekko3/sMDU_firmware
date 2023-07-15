/*
 * encoder.hpp
 *
 *  Created on: 2023/04/10
 *      Author: yaa3k
 */

#ifndef DRIVER_ENCODER_HPP_
#define DRIVER_ENCODER_HPP_

#include "board_data.hpp"
#include "motor_math.hpp"
#include "adc.hpp"


#define USE_AS5600
#ifdef USE_AS5600
//#define I2C_DMA

//AS5600//////////////////////////////////////////////////////////////////////////////
class AS5600{
private:
	I2C_HandleTypeDef *i2c;
	const uint16_t as5600_id = 0x36;
	const uint16_t resolution = 0xFFF;

	bool data_new = false;
	uint8_t enc_val[2] = {0};

public:

	AS5600(I2C_HandleTypeDef *_i2c):i2c(_i2c){}

	void init(void);
	void read_start(void);
	uint16_t get_angle(void);

	uint16_t get_resolution(void){
		return resolution;
	}

	bool is_available(void){
		return data_new;
	}
	void set_flag(bool f){
		data_new = f;
	}
};
#endif

//AB LINER HALL SENSOR(for robotmaster)///////////////////////////////////////////////
enum class HALL_SENS{
	H1,
	H2
};
class AB_LINER{
private:
	ADC &adc;


	float raw_to_regular[2]={0};
	int16_t move_to_centor[2]={0};
public:
	AB_LINER(ADC &_adc):adc(_adc){}
	void set_param(HALL_SENS sens,int16_t min,int16_t max);
	uint16_t get_raw(HALL_SENS sens){
		return adc.get_raw(sens);
	}
	sincos_t get_sincos(void);
};


//ENCODER/////////////////////////////////////////////////////////////////////////////
class ENCODER{
private:
	const int motor_pole;
	motor_math &math;
	AS5600 &as5600;
	//AS5048 &as5048;
	//ABX &abx;
	//UVW_HALL &uvw_hall;
	AB_LINER &ab_liner;
	ENC_type type;

	int16_t origin = 0;

	//for search origin
	int count = 0;
	int16_t origin_search_sum = 0;

public:
	ENCODER(int _motor_pole,motor_math &_math,AS5600 &_as5600,AB_LINER &_ab_liner)
	:motor_pole(_motor_pole),math(_math),as5600(_as5600),ab_liner(_ab_liner){

	}

	void select(ENC_type _type){
		type = _type;
	}
	void init(void);
	void calibrate(const int angle);
	void search_origin(const int angle);
	void calc_param(void);

	bool is_available(void);

	void timer_interrupt_task(void);

	int get_e_angle(void);

	sincos_t get_e_sincos(void);
};

#endif /* DRIVER_ENCODER_HPP_ */
