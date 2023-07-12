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

//AB LINER HALL SENSOR(for robotmaster)///////////////////////////////////////////////
void AB_LINER::set_param(uint16_t min,uint16_t max){
	raw_to_regular = 2/(max-min);
	move_to_centor = -(max-min)/2 - min;
}
sincos_t AB_LINER::get_sincos(void){
	sincos_t data;
	data.sin = (float)(adc.get_raw(ADC_data::RBM_H1)+move_to_centor)*raw_to_regular;
	data.cos = (float)(adc.get_raw(ADC_data::RBM_H2)+move_to_centor)*raw_to_regular;
	return data;
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
		if(angle == 0){

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
		//ab_hall.init();
		break;
	}
}
void ENCODER::calc_param(void){

}

float ENCODER::get_rad_speed(void){
	switch(type){
	case ENC_type::AS5600:
		as5600.init();
		break;
	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		//ab_hall.init();
		break;
	}
}
float ENCODER::get_rad(void){
	switch(type){
	case ENC_type::AS5600:
		as5600.init();
		break;
	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		//ab_hall.init();
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
		return ture;
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
		//as5048.init();
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

int ENCODER::get_e_angle(void){
	//e_angle = (data%(resol/pole))*(1024/(resol/pole))

	switch(type){
	case ENC_type::AS5600:
		as5600.init();
		break;
	case ENC_type::AS5048:
		//as5048.init();
		break;
	case ENC_type::ABX:
		break;
	case ENC_type::UVW_HALL:
		break;
	case ENC_type::AB_LINER_HALL:
		//ab_hall.init();
		break;
	}
}
sincos_t ENCODER::get_e_sincos(void){
	sincos_t data;
	switch(type){
	case ENC_type::AS5600:
		int angle = as5600.get_angle()-origin;
		angle = ((angle*motor_pole*0x3FF/as5600.get_resolution())&0x3FF);
		data.sin = math.sin_t(angle);
		data.cos = math.cos_t(angle);
		break;
	case ENC_type::AS5048:
		//as5048.init();
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
