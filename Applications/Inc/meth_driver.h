#pragma once
#ifndef MECH_DRIVER_H
#define MECH_DRIVER_H 

#include "include.h"

#define POS_UP_One  (-8192.0f*330.0f)
#define POS_UP_Two    (-8192.0f * 580.0f) 
#define POS_DOWN  (0.0f)

// // 电磁阀状态枚举定义
typedef enum {
    VALVE_OFF = 0,
    VALVE_ON
} Valve_State_e;

void Servo_Init_All(void);
void Servo_Set_Angle_Dual(float angle1, float angle2); // ͬ初始化全部舵机


void Pneumatic_Gripper_Ctrl(Valve_State_e state);   // /气动夹爪控制
void Pneumatic_Suction_Ctrl(Valve_State_e state);   //  气动吸盘控制

void Host_Servo_Logic(float input_val);
void Mech_Servo_Logic(float input_val, float roll_val);
void Mech_Suction_Logic(float input_val,float input_pn,float roll_val);
//void Mech_Lift_Logic(float input_val, float roll_val);
void Mech_Lift_SetTarget(float input_val, float roll_val);
void Mech_Lift_PID_Update(void);

#endif

