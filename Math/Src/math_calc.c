#include "math_calc.h"
#include <math.h>
/**
 * @brief  ����ת��Ϊ�Ƕ�
 * @param  rad ����ֵ
 * @retval deg �Ƕ�ֵ
 * @attention
 */
float rad2deg(float rad)
{
    float deg;
    deg = (float)(rad * (180.0 / PI));
    return deg;
}

//�Ƕ�ת����
float deg2rad(float angle_deg)
{
    return angle_deg * (PI / 180.0f);
}

//�ǶȻ������ֵ
float Angle_To_Encoder(double theta)
{
   if (theta < 0){theta += 360.0;}

   float encoder = 0;
   encoder = (theta / 360.0) * 8192.0;
   return encoder;
}

//����ֵ����Ƕ�(������)
float Encoder_To_Angle(float encoder)
{
    float angle = 0.0;
    angle = (encoder / 8191.0) * 2.0 * PI;
    return angle;
}

/**
 * @brief  �����Ƕȼ��㣬����������Ҫת���ĽǶ�
 * @param  vx ÿ�����ӵ�x�����ٶ�
 * @param  vy ÿ�����ӵ�y�����ٶ�
 * @retval angle �Ƕ�ֵ��0��-360�㣩
 * @attention �˺����ǽ��Ƕȹ淶��-180��-180��
 */
float Angle_Calc(float vx, float vy)
{
   const float EPS = 1e-6f;

    if(fabsf(vx) < EPS && fabsf(vy) < EPS)
  {
        return 0.0f;
    }
    /* ���ط�Χ [-180, 180] */
    float theta = atan2f(vy, vx);
    float deg = rad2deg(theta);   

    /* �� -180 ҕ�� +180 */
    if (deg <= -180.0f + EPS) deg = 180.0f;

    return deg;
}

/*�����ж�*/
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

float wrap_diff_deg(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}


/* ���Ƕȹ淶�� [-180,180] */
 float normalize_deg(float deg)
{
    /* ��һ���� (-180, 180] */
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/*�������ٶ�תΪ���ת��rpm*/
 float linear_speed_to_motor_rpm(float wheel_linear_speed)
 {
     float wheel_rev_s = wheel_linear_speed / (2 * PI * 0.062f);
     float motor_rev_s = wheel_rev_s * 19.208f;
     float motor_rpm = motor_rev_s * 60;

     return motor_rpm;
 }
 
// /*�������ٶ�תΪ���ticks/s*/
// float linear_speed_to_encoder_ticks_per_s(float wheel_linear_speed_m_s)
// {
//    float motor_rev_s = (wheel_linear_speed_m_s / (2.0f* PI *0.062f))*19.0f;
//    return motor_rev_s * (float)(1024*4);
// }
  
///*���Ʋ���*/
// float approach_float(float current, float target, int32_t max_delta)
//{   
//    if (max_delta <= 0) return target; 
//    float diff = target - current;
//    if (diff > (float)max_delta)  return current + (float)max_delta;
//    if (diff < -(float)max_delta) return current - (float)max_delta;
//    return target;
//}
static  int32_t encoder_wrap_diff(int32_t target, int32_t current)
{
    int32_t diff = target - current;
    if (diff > 4096)       diff -= 4096;
    else if (diff < -8192) diff += 8192;
    return diff;
}

// ��Ŀ�껻�ɡ��뵱ǰ������ĵȼ�Ŀ��
int32_t encoder_wrap_target(int32_t current, int32_t target)
{
    return current + encoder_wrap_diff(target, current);
}

