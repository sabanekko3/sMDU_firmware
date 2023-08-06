/*
 * motor.cpp
 *
 *  Created on: Jun 27, 2023
 *      Author: yaa3k
 */
#include "motor.hpp"

void MOTOR::init(void){
	//pwm driver set
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
}

void MOTOR::print_debug(void){

	//printf("%4.3f,%4.3f\r\n",test.d,test.q);
	//printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",i_dq.d,i_dq.q,v_dq.d,v_dq.q);
	printf("%4.3f,%4.3f,%4.3f,%5.2f,%4.3f,%4.3f\r\n",i_uvw.u,i_uvw.v,i_uvw.w,adc.get_power_v(),i_dq.d,i_dq.q);
	//printf("%4.3f,%4.3f,%4.3f,%4.3f\r\n",enc.get_e_sincos().sin,enc.get_e_sincos().cos,math.sin_t(angle_e),math.cos_t(angle_e));
	//for(int i = 0;i<(int)ADC_data::n;i++) printf("%d,",adc.get_raw(i));
	//printf("\r\n");

	//printf("%d,%d,%d\r\n",adc.get_raw(ADC_data::U_I),adc.get_raw(ADC_data::V_I));

}

PID pid_d(-0.001,-0.002,0,-0.9,0.9);
PID pid_q(-0.001,-0.002,0,-0.9,0.9);
void MOTOR::control(void){
	i_uvw = adc.get_i_uvw();

	math.dq_from_uvw(i_uvw,enc.get_e_sincos(), &i_dq);

	v_dq.d = pid_d.calc(i_dq_target.d,i_dq.d);
	v_dq.q = pid_q.calc(i_dq_target.d,i_dq.q);
//	v_dq.d = -0.1;
//	v_dq.q = 0.0;

	math.uvw_from_dq(v_dq,enc.get_e_sincos(), &v_uvw);
	driver.out(v_uvw);

//	driver.out(angle_e,0.3f);
//	angle_e += 1;
}

