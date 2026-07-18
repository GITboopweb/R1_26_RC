#pragma once
#ifndef __MATH_CALC_H
#define __MATH_CALC_H
#include "include.h"

#define PI 3.14159265358979323846f

float rad2deg(float rad);
float deg2rad(float angle_deg);
float Angle_To_Encoder(double theta);
float Encoder_To_Angle(float encoder);
float Angle_Calc(float vx, float vy);
 
 float wrap_diff_deg(float target, float current);
 float normalize_deg(float deg);
 float sign(float x);

 float linear_speed_to_motor_rpm(float wheel_linear_speed);
// float linear_speed_to_encoder_ticks_per_s(float wheel_linear_speed_m_s);
// float approach_float(float current, float target, int32_t max_delta);
int32_t encoder_wrap_target(int32_t current, int32_t target);

#endif // !__MATH_CALC_H
