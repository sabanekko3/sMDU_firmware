/*
 * motor_math.cpp
 *
 *  Created on: 2023/04/03
 *      Author: yaa3k
 */


#include "../driver/motor_math.hpp"

float motor_math::table[TABLE_SIZE]={0};

motor_math::motor_math(void){
	for (int i = 0; i < TABLE_SIZE; i++) {
		float rad = (float)i/(float)TABLE_SIZE * 2*M_PI;
#ifdef USE_CMSIS
		table[i] = arm_sin_f32(rad);
#else
		table[i] = sin(rad);
#endif

	}
}

void motor_math::dq_from_uvw(uvw_t input,uint16_t angle_e,dq_t *out){
	float sin = sin_table(angle_e);
	float cos = cos_table(angle_e);

	//clarke
	ab_t ab_data;
	arm_clarke_f32(input.u,input.v,&ab_data.a,&ab_data.b);
	//park
	arm_park_f32(ab_data.a,ab_data.b,&(out->d),&(out->q),sin,cos);
}

void motor_math::dq_from_uvw(uvw_t input,sincos_t param,dq_t *out){
	//clarke
	ab_t ab_data;
	arm_clarke_f32(input.u,input.v,&ab_data.a,&ab_data.b);
	//park
	arm_park_f32(ab_data.a,ab_data.b,&(out->d),&(out->q),param.sin,param.cos);
}

void motor_math::uvw_from_dq(dq_t input,uint16_t angle_e,uvw_t *out){
	float sin = sin_table(angle_e);
	float cos = cos_table(angle_e);

	//inv park
	ab_t ab_data;
	arm_inv_park_f32(input.d,input.q,&ab_data.a,&ab_data.b,sin,cos);

	//inv clarke
	arm_inv_clarke_f32(ab_data.a,ab_data.b,&(out->u),&(out->v));
	out->w = -out->u - out->v;
}


void motor_math::uvw_from_dq(dq_t input,sincos_t param,uvw_t *out){
	//inv park
	ab_t ab_data;
	arm_inv_park_f32(input.d,input.q,&ab_data.a,&ab_data.b,param.sin,param.cos);

	//inv clarke
	arm_inv_clarke_f32(ab_data.a,ab_data.b,&(out->u),&(out->v));
	out->w = -out->u - out->v;
}

float motor_math::fast_atan2_rad(float _y,float _x){
    float x = abs(_x);
    float y = abs(_y);
    float xy_ratio;
    bool x_is_large;

    x_is_large = y < x;

    if (x_is_large){
        xy_ratio = y / x;
    }else{
        xy_ratio = x / y;
    }

    float rad = xy_ratio * ( xy_ratio * ( xy_ratio * (0.144614 *  xy_ratio - 0.3508) - 0.010117) + 1);
    //float rad = xy_ratio * ( xy_ratio * (-0.07815 * xy_ratio - 0.16642) + 1.028176);

    if (x_is_large) {
        if (_x > 0) {
            if (_y < 0)rad *= -1;
        }
        if (_x < 0) {
            if (_y > 0)rad = M_PI - rad;
            if (_y < 0)rad = rad - M_PI;
        }
    }

    if (!x_is_large) {
        if (_x > 0) {
            if (_y > 0) rad = M_PI_2 - rad;
            if (_y < 0) rad = rad - M_PI_2;
        }
        if (_x < 0) {
            if (_y > 0) rad = rad + M_PI_2;
        if (_y < 0) rad = -rad - M_PI_2;
        }
    }
  return rad;
}

//PID/////////////////////////////////////////////////////////////////////////
float PID::calc(float target,float feedback){
	error = target - feedback;
	float p = error * kp;

	float error_sum_old = error_sum;
	error_sum += error;
	float i = error_sum * ki;

	float d = (error - old_error) * kd;
	old_error = error;

	float out = p+i+d;


	if(limit_enable){
		if(out < out_min){
			out = out_min;
			error_sum = error_sum_old;
		}
		if(out_max < out){
			out = out_max;
			error_sum = error_sum_old;
		}
	}
	return out;
}
