/*
 * motor_math.hpp
 *
 *  Created on: 2023/04/03
 *      Author: yaa3k
 */

#ifndef DRIVER_MOTOR_MATH_HPP_
#define DRIVER_MOTOR_MATH_HPP_

#include "board_data.hpp"

#define TABLE_SIZE 1024
#define COSP 256
#define V_PHASE 341
#define W_PHASE 683

constexpr float angle_to_rad = (2*M_PI)/TABLE_SIZE;
constexpr float rad_to_angle = TABLE_SIZE/(2*M_PI);

typedef struct dq{
	float d;
	float q;
}dq_t;
typedef struct ab{
	float a;
	float b;
}ab_t;
typedef struct uvw{
	float u;
	float v;
	float w;
}uvw_t;
typedef struct sincos{
	float sin;
	float cos;
}sincos_t;


#define USE_CMSIS
#ifdef USE_CMSIS
	#define ARM_MATH_CM4
	#include "arm_math.h"
	#include "arm_const_structs.h"
#endif


class motor_math{
private:
#ifndef USE_CMSIS
	float sqrt3 = sqrt(3.0);
	float sqrt3inv = 1/sqrt(3.0);
#endif
	const float angle_rad = 0x3FF/(2*M_PI);
	float table[TABLE_SIZE];
public:
	motor_math(void);
	float sin_t(int angle){
		return table[angle & 0x3FF];
	}
	float cos_t(int angle){
		return table[(angle+COSP) & 0x3FF];
	}
	float sin_t(float rad){
		return table[(uint16_t)(rad * angle_rad) & 0x3FF];
	}
	float cos_t(float rad){
		return table[((uint16_t)(rad * angle_rad)+COSP) & 0x3FF];
	}

	void dq_from_uvw(uvw_t input,uint16_t deg_e,dq_t *out);
	void dq_from_uvw(uvw_t input,sincos_t param,dq_t *out);
	void uvw_from_dq(dq_t input,uint16_t deg_e,uvw_t *out);
	void uvw_from_dq(dq_t input,sincos_t param,uvw_t *out);
};

class PID{
private:
	float kp = 0;
	float ki = 0;
	float kd = 0;
	float error = 0;
	float error_sum = 0;
	float old_error = 0;
public:
	PID(float _kp,float _ki,float _kd):kp(_kp),ki(_ki),kd(_kd){}
	float compute(float target,float feedback);

	void reset(void){
		error = 0;
		error_sum = 0;
		old_error = 0;
	};
};


class LPF{
private:
	float data = 0;
	float k = 0;
public:
	LPF(float _k):k(_k){}
	float solve(float input){
		data = input*k+(1.0-k)*data;
		return data;
	}
	void reset(void){
		data = 0;
	}
};

class HPF{
private:
	float data = 0;
	float k = 0;
public:
	HPF(float _k):k(_k){}
	float solve(float input){
		data = input*k+(1.0-k)*data;
		return input - data;
	}
	void reset(void){
		data = 0;
	}
};


#endif /* DRIVER_MOTOR_MATH_HPP_ */
