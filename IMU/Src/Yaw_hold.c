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

  // �ֶ����ٶȳ����������˳����򱣳�
	if (fabsf(omega_user) > YAW_HOLD_OMEGA_DEADBAND)
    {
        g_yaw_hold.active = 0;
        g_yaw_hold.i_term = 0.0f;
        return omega_user;
    }

    // ƽ���ٶȳ����ƶ�����������������ʻ״̬���������򱣳�������ͷ����
    if (trans_mag > YAW_HOLD_MOVE_DEADBAND)
    {
        if (!g_yaw_hold.active)
        {
            g_yaw_hold.active = 1;
            g_yaw_hold.target_yaw_deg = yaw_now_deg;   // ������ǰ����ΪĿ��Ƕ�
            g_yaw_hold.i_term = 0.0f;
        }

        float err = wrap_diff_deg(g_yaw_hold.target_yaw_deg, yaw_now_deg);

        // PID�������㣺������ + ������ + ������΢����
        g_yaw_hold.i_term += YAW_HOLD_KI * err * dt;
        g_yaw_hold.i_term = LIMIT(g_yaw_hold.i_term, -YAW_HOLD_I_MAX, YAW_HOLD_I_MAX);

        float omega_hold = YAW_HOLD_KP * err + g_yaw_hold.i_term - YAW_HOLD_KD * gyro_z_dps;
        omega_hold = LIMIT(omega_hold, -YAW_HOLD_OMEGA_MAX, YAW_HOLD_OMEGA_MAX);

        /* ���ٽ�Ȩ����������ǿ����ƽ���ٶ��������� */
        /* �������ʱ��������ռ�ȹ��ߣ���������ת�����ͻأ�����̶��� */
        const float YAW_FULL_SCALE_SPEED = 1.8f;   /* �ﵽ���ٶȼ�����ʱ�����򱣳���Ȩ����� */
        float spd_scale = trans_mag / YAW_FULL_SCALE_SPEED;
        if (spd_scale > 1.0f) spd_scale = 1.0f;
        omega_hold *= spd_scale;

        return omega_hold;
    }

    // ƽ���ٶȵ��������������ú��򱣳�
    g_yaw_hold.active = 0;
    g_yaw_hold.i_term = 0.0f;
    return 0.0f;
}
