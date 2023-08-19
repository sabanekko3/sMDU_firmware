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
#include "analog_sens.hpp"


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


	int turn_count = 0;
	uint16_t angle_old = 0;

public:
	AS5600(I2C_HandleTypeDef *_i2c):i2c(_i2c){}

	void init(void);
	void read_start(void);
	void check_turn(void);

	//get parameter
	uint16_t get_raw(void){
		return (enc_val[0]<<8)|enc_val[1];
	}
	uint16_t get_resolution(void){
		return resolution;
	}
	bool is_available(void){
		return data_new;
	}
	void set_flag(bool f){
		data_new = f;
	}
	int get_count(void){
		return turn_count;
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
	ANALOG_SENS &analog;

	//parametor for get_sincos
	float raw_to_regular[2]={0};
	int16_t move_to_centor[2]={0};

	int32_t turn_count = 0;
	uint32_t enc_phase_log = 0;
public:
	AB_LINER(ANALOG_SENS &_analog):analog(_analog){}

	void set_param(HALL_SENS sens,int16_t min,int16_t max);

	sincos_t get_sincos(void);

	//for timer int
	void check_turn(sincos_t sincos_data);


	//inline functions
	uint16_t get_raw(HALL_SENS sens){
		return analog.get_raw(sens);
	}
	int get_turn_count(void){
		return turn_count;
	}
};


//ENCODER/////////////////////////////////////////////////////////////////////////////
class ENCODER:motor_math{
private:
	const int motor_pole;

	//encoders
	AS5600 &as5600;
	//AS5048 &as5048;
	//ABX &abx;
	//UVW_HALL &uvw_hall;
	AB_LINER &ab_liner;
	ENC_type type;

	int16_t origin = 0;

	//for search origin
	int32_t calibration_count = 0;
	int32_t origin_search_sum = 0;

	uint16_t angle_old = 0;


public:
	ENCODER(int _motor_pole,AS5600 &_as5600,AB_LINER &_ab_liner)
	:motor_pole(_motor_pole),as5600(_as5600),ab_liner(_ab_liner){

	}

	void select(ENC_type _type){
		type = _type;
	}
	void init(void);
	void calibrate(const int angle);
	void calc_param(void);

	bool is_available(void);

	void timer_interrupt_task(void);
	void read_completion_interrupt_task(void);

	//e_angle:電気角
	uint16_t get_e_angle(void);
	int32_t get_e_angle_sum(void);

	sincos_t get_e_sincos(void);
};

#endif /* DRIVER_ENCODER_HPP_ */
