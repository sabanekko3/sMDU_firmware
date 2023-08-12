/*
 * motor.cpp
 *
 *  Created on: Jun 27, 2023
 *      Author: yaa3k
 */
#include "motor.hpp"


void MOTOR::init(void){
	//pwm driver setting
	driver.pwms_start();

	//adc start
	adc.init();
	adc.dma_start();

	//encoder reset
	enc.init();

	//encoder calibration///////////////
	//move to origin
	for(float f = 0; f<0.2f; f+=0.001f){
		driver.out(0,f);
		HAL_Delay(1);
	}
	driver.out(0,0.2f);
	HAL_Delay(300);
	enc.search_origin(0);

	//turn foward 360deg
	for(int i = 0; i < 0x1FF; i++){
		driver.out(i,0.1f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0x200,0.1f);
	HAL_Delay(300);
	//enc.search_origin(0x400);

	//turn reverse 360deg
	for(int i = 0x1FF; i >= 0; i--){
		driver.out(i,0.1f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,0.1f);
	HAL_Delay(300);
	enc.search_origin(0);

	//turn reverse 360deg
	for(int i = 0; i >= -0x1FF; i--){
		driver.out(i,0.1f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(-0x200,0.1f);
	HAL_Delay(300);
	//enc.search_origin(-0x400);

	//turn foward 360deg
	for(int i = -0x1FF; i < 0; i++){
		driver.out(i,0.1f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,0.1f);
	HAL_Delay(300);
	enc.search_origin(0);

	enc.calc_param();

	//gain setting
	pid_d.set_gain(0.01,5,0);
	pid_q.set_gain(0.01,5,0);
}

void MOTOR::print_debug(void){
	sincos_t sincos_val = enc.get_e_sincos();
	//printf("%4.3f,%4.3f\r\n",test.d,test.q);
	//printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",i_dq.d,i_dq.q,v_dq.d,v_dq.q);
	//printf("%4.3f,%4.3f,%4.3f,%5.2f,%4.3f,%4.3f\r\n",i_uvw.u,i_uvw.v,i_uvw.w,adc.get_power_v(),i_dq.d,i_dq.q);
	//printf("%4.3f,%4.3f,%4.3f\r\n",i_uvw.u,i_uvw.v,i_uvw.w);
	//printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",enc.get_e_sincos().sin,enc.get_e_sincos().cos,math.sin_t(angle_e),math.cos_t(angle_e));
	//for(int i = 0;i<(int)ADC_data::n;i++) printf("%d,",adc.get_raw(i));
	//printf("%d\r\n",enc.get_e_angle_sum());
	//printf("%d\r\n",enc.get_e_angle());
	//printf("%d,%d,%d\r\n",adc.get_raw(ADC_data::U_I),adc.get_raw(ADC_data::V_I));
	//printf("%4.3f\r\n",i_dq_target.q);
	//printf("%4.3f,%4.3f,%4.3f\r\n",sincos_val.sin,sincos_val.cos,fast_atan2_rad(sincos_val.sin,sincos_val.cos));
	printf("%4.3f,%4.3f,%d\r\n",sincos_val.sin,sincos_val.cos,angle_e);
}


void MOTOR::control(void){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,GPIO_PIN_SET);
	i_uvw = adc.get_i_uvw();

	sincos_t enc_sincos = enc.get_e_sincos();

	dq_from_uvw(i_uvw,enc_sincos, &i_dq);

	v_dq.d = pid_d.calc(i_dq_target.d,i_dq.d);
	v_dq.q = pid_q.calc(i_dq_target.q,i_dq.q);
//	v_dq.d = 0.0f;
//	v_dq.q = 0.05f;

	uvw_from_dq(v_dq,enc_sincos, &v_uvw);
	driver.out(v_uvw);

	angle_e = fast_atan2_angle(enc_sincos.sin,enc_sincos.cos);
//	static float _angle = 0;
//	driver.out(angle_e,0.05f);
//	_angle += 1;
//	angle_e = (int)_angle;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_13,GPIO_PIN_RESET);
}

