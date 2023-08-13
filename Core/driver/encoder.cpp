/*
 * encoder.cpp
 *
 *  Created on: 2023/04/11
 *      Author: yaa3k
 */

#include "encoder.hpp"


//AS5600////////////////////////////////////////////////////////////sss//////////////////
void AS5600::init(void){
	uint8_t txdata = 0x0c;
	HAL_I2C_Master_Transmit(i2c, as5600_id<<1, &txdata, 1, 100);
}

void AS5600::read_start(void){
	data_new = false;
	HAL_I2C_Master_Receive_IT(i2c, as5600_id<<1, enc_val, 2);
}

uint16_t AS5600::get_angle(void){
	return (enc_val[0]<<8)|enc_val[1];
}

void AS5600::check_turn(){
	uint16_t angle = get_angle();

	if((angle > (resolution>>2)) && (angle_old > (resolution>>2)*3)){
		turn_count ++;
	}else if((angle > (resolution>>2)*3) && (angle_old > (resolution>>2))){
		turn_count --;
	}
	angle_old = angle;
}
//AB LINER HALL SENSOR(for robotmaster)///////////////////////////////////////////////
void AB_LINER::set_param(HALL_SENS sens,int16_t min,int16_t max){
	raw_to_regular[(int)sens] = 2.0f/(float)(max-min);
	move_to_centor[(int)sens] = -(max-min)/2 - min;
}

sincos_t AB_LINER::get_sincos(void){
	sincos_t data;
	data.sin = -(float)(adc.get_raw(ADC_data::RBM_H2)+move_to_centor[(int)HALL_SENS::H2])*raw_to_regular[(int)HALL_SENS::H2];
	data.cos = (float)(adc.get_raw(ADC_data::RBM_H1)+move_to_centor[(int)HALL_SENS::H1])*raw_to_regular[(int)HALL_SENS::H1];
	return data;
}

void AB_LINER::check_turn(void){
	sincos_t sincos_data = get_sincos();

	int enc_phase = 0;
	enc_phase |= (sincos_data.sin >= 0.0f) ? 0b01:0b00;
	enc_phase |= (sincos_data.cos >= 0.0f) ? 0b10:0b00;

	if(enc_phase == 0b10 && enc_phase_log == 0b11){
		turn_count --;
	}else if(enc_phase == 0b11 && enc_phase_log == 0b10){
		turn_count ++;
	}
	enc_phase_log = enc_phase;
}

///////////////////////////////////////////////////////////////////////////////
void ENCODER::init(void){
	switch(type){
	case ENC_type::AS5600:
		as5600.init();
		break;
	case ENC_type::AS5048:

		break;
	case ENC_type::ABX:

		break;
	case ENC_type::UVW_HALL:

		break;
	case ENC_type::AB_LINER_HALL:
		//nop
		break;
	}
}

void ENCODER::calibrate(const int angle){
	switch(type){
	case ENC_type::AS5600:
		//nop
		break;
	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		static uint16_t max[2] = {0,0};
		static uint16_t min[2] = {6000,6000};
		uint16_t h1 = ab_liner.get_raw(HALL_SENS::H2);
		uint16_t h2 = ab_liner.get_raw(HALL_SENS::H2);

		if(max[(int)HALL_SENS::H1] < h1) max[(int)HALL_SENS::H1] = h1;
		if(max[(int)HALL_SENS::H2] < h2) max[(int)HALL_SENS::H2] = h2;

		if(min[(int)HALL_SENS::H1] > h1) min[(int)HALL_SENS::H1] = h1;
		if(min[(int)HALL_SENS::H2] > h2) min[(int)HALL_SENS::H2] = h2;

		ab_liner.set_param(HALL_SENS::H1,min[(int)HALL_SENS::H1], max[(int)HALL_SENS::H1]);
		ab_liner.set_param(HALL_SENS::H2,min[(int)HALL_SENS::H2], max[(int)HALL_SENS::H2]);
		break;
	}
}
void ENCODER::search_origin(int angle){
	switch(type){
	case ENC_type::AS5600:

		if(angle == 0){
			as5600.read_start();
			while(!as5600.is_available());
			int16_t angle = as5600.get_angle() - origin;
			if(origin_search_count == 0){
				origin = angle;
			}else{
				angle |= (angle&0x200)?0xFC00:0;
				origin_search_sum += angle;
			}
		}
		break;

	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		break;
	}
	origin_search_count ++;
}
void ENCODER::calc_param(void){
	origin_search_count = 0;
	switch(type){
	case ENC_type::AS5600:

		if(origin_search_count != 0){
			origin += origin_search_sum / origin_search_count;
		}

		break;
	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		break;
	}
}


bool ENCODER::is_available(void){
	switch(type){
	case ENC_type::AS5600:
		return as5600.is_available();
		break;
	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		return true;
		break;
	}
	return false;
}
void ENCODER::timer_interrupt_task(void){
	switch(type){
	case ENC_type::AS5600:
		as5600.read_start();
		break;
	case ENC_type::AS5048:
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		ab_liner.check_turn();
		break;
	}
}
void ENCODER::read_completion_interrupt_task(void){
	switch(type){
	case ENC_type::AS5600:
		as5600.set_flag(true);
		as5600.check_turn();
		break;
	case ENC_type::AS5048:
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		//nop
		break;
	}
}

uint16_t ENCODER::get_e_angle(void){
	int angle = 0;
	int rad = 0.0f;
	sincos_t sincos_data;

	switch(type){
	case ENC_type::AS5600:

		angle = (as5600.get_angle()-origin)&0x3FF;
		return ((angle*motor_pole*0x3FF/as5600.get_resolution())&0x3FF);
		break;

	case ENC_type::AS5048:
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		sincos_data = ab_liner.get_sincos();
		return fast_atan2_angle(sincos_data.sin,sincos_data.cos);
		break;
	}
	return 0;
}

int32_t ENCODER::get_e_angle_sum(void){
	int div_tmp;
	switch(type){
	case ENC_type::AS5600:
		div_tmp = (as5600.get_angle()*motor_pole)/as5600.get_resolution();
		return (as5600.get_count()*motor_pole+div_tmp)*1024*motor_pole + get_e_angle();
		break;
	case ENC_type::AS5048:
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		return ab_liner.get_turn_count() * 1024 + get_e_angle();
		break;
	}
	return 0;
}

sincos_t ENCODER::get_e_sincos(void){
	sincos_t data;
	int angle = 0;
	switch(type){
	case ENC_type::AS5600:
		angle = (as5600.get_angle()-origin)&0x3FF;

		//mechanical angle -> electrical angle
		angle = ((angle*motor_pole*0x3FF/as5600.get_resolution())&0x3FF);

		data.sin = sin_table(angle);
		data.cos = cos_table(angle);
		break;

	case ENC_type::AS5048:
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:

		data = ab_liner.get_sincos();
		break;
	}

	return data;
}
