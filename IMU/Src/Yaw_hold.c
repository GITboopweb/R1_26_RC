#include "Yaw_hold.h"

YawHold_t g_yaw_hold;


void YawHold_Init(void)
{ 
    g_yaw_hold.target_yaw_deg = 0.0f;
    g_yaw_hold.active = 0;
    g_yaw_hold.i_term = 0.0f;
}

float YawHold_Update(float vx_cmd, float vy_cmd, float omega_user,float yaw_now_deg, float gyro_z_dps, float dt)
{
    float trans_mag = hypotf(vx_cmd, vy_cmd);

    // 用户给定角速度超出死区，关闭航向保持功能、清空积分项
	if (fabsf(omega_user) > YAW_HOLD_OMEGA_DEADBAND)
    {
        g_yaw_hold.active = 0;
        g_yaw_hold.i_term = 0.0f;
        return omega_user;
    }

    // 底盘有平移速度，进入航向保持逻辑
    if (trans_mag > YAW_HOLD_MOVE_DEADBAND)
    {
        if (!g_yaw_hold.active)
        {
            g_yaw_hold.active = 1;
            g_yaw_hold.target_yaw_deg = yaw_now_deg;   // 锁定当前航向为目标角度
            g_yaw_hold.i_term = 0.0f;
        }

        float err = wrap_diff_deg(g_yaw_hold.target_yaw_deg, yaw_now_deg);

        // 航向保持PID运算：比例项 + 积分项 - 微分项（陀螺仪角速度做微分反馈）
        g_yaw_hold.i_term += YAW_HOLD_KI * err * dt;
        g_yaw_hold.i_term = LIMIT(g_yaw_hold.i_term, -YAW_HOLD_I_MAX, YAW_HOLD_I_MAX);

        float omega_hold = YAW_HOLD_KP * err + g_yaw_hold.i_term - YAW_HOLD_KD * gyro_z_dps;
        omega_hold = LIMIT(omega_hold, -YAW_HOLD_OMEGA_MAX, YAW_HOLD_OMEGA_MAX);

        /* 平移速度加权缩放：底盘移动越快，航向保持力度越强 */
        /* 避免高速平移时航向修正力度不足、低速时修正抖动过大 */
        const float YAW_FULL_SCALE_SPEED = 1.8f;   /* 达到该平移速度时，航向保持输出满权值 */
        float spd_scale = trans_mag / YAW_FULL_SCALE_SPEED;
        if (spd_scale > 1.0f) spd_scale = 1.0f;
        omega_hold *= spd_scale;

        return omega_hold;
    }

    // 无平移速度，关闭航向保持、清空积分，直接返回零角速度
    g_yaw_hold.active = 0;
    g_yaw_hold.i_term = 0.0f;
    return 0.0f;
}
