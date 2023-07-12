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

	//turn foward 180deg
	for(int i = 0; i < 0x1FF; i++){
		driver.out(i,0.2f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0x200,0.2f);
	HAL_Delay(300);
	enc.search_origin(0x200);

	//turn reverse 180deg
	for(int i = 0x200; i >= 0; i--){
		driver.out(i,0.2f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,0.2f);
	HAL_Delay(300);
	enc.search_origin(0);

	//turn reverse 180deg
	for(int i = 0; i >= -0x200; i--){
		driver.out(i,0.2f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,0.2f);
	HAL_Delay(300);
	enc.search_origin(-0x200);

	//turn foward 180deg
	for(int i = -0x200; i < 0; i++){
		driver.out(i,0.2f);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0x200,0.2f);
	HAL_Delay(300);
	enc.search_origin(0);

	enc.calc_param();
}

void MOTOR::print_debug(void){
	uint32_t pwm[3];
	driver.get_pwm_val(&pwm[0], &pwm[1],&pwm[2]);
//	i_uvw.u = i_uvw.u<0 ? i_uvw.u/((float)pwm[0]*0.001f) : i_uvw.u;
//	i_uvw.v = i_uvw.v<0 ? i_uvw.v/((float)pwm[1]*0.001f) : i_uvw.v;
//	i_uvw.w = i_uvw.w<0 ? i_uvw.w/((float)pwm[2]*0.001f) : i_uvw.w;
	//printf("%4.3f,%4.3f,%4.3f,%5.2f,%4.3f,%4.3f\r\n",i_uvw.u,i_uvw.v,i_uvw.w,adc.get_power_v(),i_dq.d,i_dq.q);
	printf("%d,%d\r\n",adc.get_raw(ADC_data::RBM_H1),adc.get_raw(ADC_data::RBM_H2));

	math.dq_from_uvw(i_uvw, angle_m, &i_dq);
	//printf("%4.3f,%4.3f,%d,%d\r\n",i_dq.d,i_dq.q,angle_m,angle_e&0x3FF);
	//printf("%d\r\n",angle_m);
}

PID pid_d(0.001f,0.001f,0);
PID pid_q(0.001f,0.001f,0);
void MOTOR::control(void){
	i_uvw = adc.get_i_uvw();


	math.dq_from_uvw(i_uvw, angle_m, &i_dq);
	v_dq.d = pid_d.compute(0.0f,i_dq.d);
	v_dq.q =pid_q.compute(0.05f,i_dq.q);

	math.uvw_from_dq(v_dq, angle_m, &v_uvw);
	//driver.out(v_uvw);

	//angle_e = angle_m + COSP;
	driver.out(angle_e,0.05f);
	angle_e += 1;
}

