#pragma once
#ifndef __TOOL_CALC_H
#define __TOOL_CALC_H

/* --- �궨�岹�� --- */
#define RC_MAX_VALUE       660.0f  // ң����������
#define RC_DEADZONE_RATIO  0.07f   // 5% ������ (��Լ��Ӧ������33)
#define RC_EXPO_RATIO      0.3f    // ����ǿ�� (0.0�����ԣ�1.0�Ǵ����η�������0.2~0.4)
#define RC_EXPO_RATIO_OMEGA   0.5f 

#include "include.h"

typedef struct {
    float prev_filtered_norm_val; 
} rc_channel_filter_state_t;

// PID���ýṹ��
typedef struct
{
    // ���ι滮�յ���������
    float arrival_window;
    // ��Ħ������������ֵ
    float static_friction_threshold;
    // ��Ħ����������ֵ
    float static_friction_current;
    // �ɱ��������ϵ��
    float vint_scale_factor;
    // �ɱ����scale����
    float vint_max_scale;
    // D���ͨ�˲�ϵ��
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

// Ϊÿ��ͨ������һ����̬ʵ�����������ǵ�״̬�ᱻ����
static rc_channel_filter_state_t rc_channel_states[3] = {0}; // �����С������ʵ���õ�ͨ��������

float apply_deadzone(float v, float dz);
float apply_expo(float v, float expo);
float rc_process_channel(int16_t raw_input, float max_phys_val, float set_expo, float lpf_alpha, uint8_t channel_idx);

float Calc_Steering_Penalty(float err_deg, float chassis_speed);
float Get_Max_Steering_Error(void);

float circular_error(float a, float b, float half_range, float full_range);
void update_trapezoidal_planner(pid_type_def *pid, float set, float dt,float half_range, float full_range);
float compute_pid_output(pid_type_def *pid, float ref, float dt,float half_range, float full_range) ;


#endif // !__TOOL_CALC_H
