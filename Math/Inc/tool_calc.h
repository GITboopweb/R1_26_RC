#pragma once
#ifndef __TOOL_CALC_H
#define __TOOL_CALC_H

/* --- 遥控器通道参数宏定义 --- */
#define RC_MAX_VALUE       660.0f  // 遥控器摇杆最大输出量程
#define RC_DEADZONE_RATIO  0.07f   // 7% 摇杆死区比例 (对应实际输出阈值约33)
#define RC_EXPO_RATIO      0.3f    // 平移通道曲线指数 (0.0线性输出，1.0曲线最强，竞赛常用0.2~0.4)
#define RC_EXPO_RATIO_OMEGA   0.5f // 旋转通道曲线指数

#include "include.h"

typedef struct {
    float prev_filtered_norm_val; 
} rc_channel_filter_state_t;

// PID控制器高级配置参数结构体
typedef struct
{
    // 位置到位误差窗口阈值
    float arrival_window;
    // 静摩擦补偿死区阈值
    float static_friction_threshold;
    // 静摩擦补偿输出值
    float static_friction_current;
    // 积分抗饱和缩放系数
    float vint_scale_factor;
    // 积分缩放系数上限
    float vint_max_scale;
    // 微分项一阶低通滤波系数
    float d_filter_alpha;
} PID_ConfigTypeDef;

#define PID_DEFAULT_CONFIG \
{           \
     5.0f, \
     10.0f,  \
     50.0f, \
     10.0f, \
     10.0f, \
     0.5f   \
}
extern PID_ConfigTypeDef pid_config;

static rc_channel_filter_state_t rc_channel_states[3] = {0}; 

float apply_deadzone(float v, float dz);
float apply_expo(float v, float expo);
float rc_process_channel(int16_t raw_input, float max_phys_val, float set_expo, float lpf_alpha, uint8_t channel_idx);

float Calc_Steering_Penalty(float err_deg, float chassis_speed);
float Get_Max_Steering_Error(void);

float circular_error(float a, float b, float half_range, float full_range);
void update_trapezoidal_planner(pid_type_def *pid, float set, float dt,float half_range, float full_range);
float compute_pid_output(pid_type_def *pid, float ref, float dt,float half_range, float full_range) ;


#endif // !__TOOL_CALC_H
