/*
 * motor.cpp
 *
 *  Created on: Jun 27, 2023
 *      Author: yaa3k
 */
#include "motor.hpp"

extern TIM_HandleTypeDef htim17;


void MOTOR::init(void){
	//pwm driver setting
	driver.pwms_start();

	//adc start
	adc.init();
	adc.dma_start();

	//encoder reset
	enc.init();

	enc_calibration(0.1);
	motor_R = measure_R(0.5);
	motor_L = measure_L(motor_R,0.5);

	printf("R:%f[Ohm],L:%f[uH]\r\n",motor_R,motor_L*1000000);

	//gain setting
	pid_d.set_gain(0.01,5,0);
	pid_q.set_gain(0.01,5,0);
}

void MOTOR::print_debug(void){
	//sincos_t sincos_val = enc.get_e_sincos();
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
	//printf("%4.3f,%4.3f,%d\r\n",sincos_val.sin,sincos_val.cos,angle_e);

	//printf("%d\r\n",__HAL_TIM_GET_COUNTER(&htim17));
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


void MOTOR::enc_calibration(float duty){
	//encoder calibration///////////////
	//move to origin
	for(float f = 0; f<duty*2; f+=0.001f){
		driver.out(0,f);
		HAL_Delay(1);
	}
	driver.out(0,duty);
	HAL_Delay(300);
	enc.search_origin(0);

	//turn foward 360deg
	for(int i = 0; i < TABLE_SIZE/2; i++){
		driver.out(i,duty);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(TABLE_SIZE/2,duty);
	HAL_Delay(300);
	//enc.search_origin(0x400);

	//turn reverse 360deg
	for(int i = TABLE_SIZE/2; i >= 0; i--){
		driver.out(i,duty);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,duty);
	HAL_Delay(300);
	enc.search_origin(0);

	//turn reverse 360deg
	for(int i = 0; i >= -TABLE_SIZE/2; i--){
		driver.out(i,duty);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(-TABLE_SIZE/2,0.1f);
	HAL_Delay(300);
	//enc.search_origin(-0x400);

	//turn foward 360deg
	for(int i = -TABLE_SIZE/2; i < 0; i++){
		driver.out(i,duty);
		enc.calibrate(i);
		HAL_Delay(1);
	}
	driver.out(0,duty);
	HAL_Delay(300);
	enc.search_origin(0);

	enc.calc_param();

	driver.out(0,0);
}

float MOTOR::measure_R(float duty){

	driver.out({duty,0,0});
	HAL_Delay(100);

	float R = 0.0f;
	for(int i = 0; i < 16; i++){
		R += (adc.get_power_v()*(duty*0.5)) /adc.get_i_uvw().u;
		HAL_Delay(1);
	}

	R *= (2.0/3.0) / 16.0;
	driver.out({0,0,0});
	return R;
}



float MOTOR::measure_L(float R,float duty){
	driver.out({0,0,0});
	HAL_Delay(100);


	R *= 3/2;
	float i_th = (adc.get_power_v()*duty*0.5)/R * (1-1/M_E) + adc.get_i_uvw().u;

	//limit:50ms
	uint16_t timer_count_limit = 50 * 1000;
	uint16_t end_count = 0;

	driver.out({duty,0,0});
	__HAL_TIM_SET_COUNTER(&htim17,0);

	while(adc.get_i_uvw().u < i_th){
		end_count = __HAL_TIM_GET_COUNTER(&htim17);

		if(timer_count_limit < end_count){
			break;
		}
	}

	float L = (float)end_count * 1.0e-06 * R;

	driver.out({0,0,0});

	return L*2/3;
}

