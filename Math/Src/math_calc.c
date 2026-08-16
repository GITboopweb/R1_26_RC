#include "math_calc.h"
#include <math.h>

/**
 * @brief Convert radians to degrees
 * @param rad Radian value
 * @retval deg Degree value
 * @attention
 */
float rad2deg(float rad)
{
    float deg;
    deg = (float)(rad * (180.0 / PI));
    return deg;
}

// Convert degrees to radians
float deg2rad(float angle_deg)
{
    return angle_deg * (PI / 180.0f);
}

// Convert degree angle to encoder tick value
float Angle_To_Encoder(double theta)
{
   if (theta < 0){theta += 360.0;}

   float encoder = 0;
   encoder = (theta / 360.0) * 8192.0;
   return encoder;
}

// Convert encoder tick to radian angle
float Encoder_To_Angle(float encoder)
{
    float angle = 0.0;
    angle = (encoder / 8191.0) * 2.0 * PI;
    return angle;
}

/**
 * @brief Calculate target angle from velocity components
 * @param vx X component of velocity
 * @param vy Y component of velocity
 * @retval deg Angle value in range [0, 360)
 * @attention atan2f native output range is [-180, 180]
 */
float Angle_Calc(float vx, float vy)
{
   const float EPS = 1e-6f;

    if(fabsf(vx) < EPS && fabsf(vy) < EPS)
  {
        return 0.0f;
    }
    /* Raw output range [-180, 180] */
    float theta = atan2f(vy, vx);
    float deg = rad2deg(theta);

    /* Map from -180 to +180 */
    if (deg <= -180.0f + EPS) deg = 180.0f;

    return deg;
}

/* Sign function */
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

/**
 * @brief Compute angle difference, wrapped within [-180, 180]
 * @param target Target angle(deg)
 * @param current Current angle(deg)
 * @return Shortest angle error
 */
float wrap_diff_deg(float target, float current)
{
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

/* Normalize degree angle into range (-180, 180] */
 float normalize_deg(float deg)
{
    /* Normalize to (-180, 180] */
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

/* Convert wheel linear speed to motor RPM */
 float linear_speed_to_motor_rpm(float wheel_linear_speed)
 {
     float wheel_rev_s = wheel_linear_speed / (2 * PI * 0.062f);
     float motor_rev_s = wheel_rev_s * 19.208f;
     float motor_rpm = motor_rev_s * 60;

     return motor_rpm;
 }

// /* Convert linear speed to encoder ticks per second */
// float linear_speed_to_encoder_ticks_per_s(float wheel_linear_speed_m_s)
// {
//    float motor_rev_s = (wheel_linear_speed_m_s / (2.0f* PI *0.062f))*19.0f;
//    return motor_rev_s * (float)(1024*4);
// }

// /* Float value slew rate limit */
// float approach_float(float current, float target, int32_t max_delta)
//{
//    if (max_delta <= 0) return target;
//    float diff = target - current;
//    if (diff > (float)max_delta)  return current + (float)max_delta;
//    if (diff < -(float)max_delta) return current - (float)max_delta;
//    return target;
//}

/**
 * @brief Calculate shortest wrapped difference for 8192‑tick encoder
 * @param target Target encoder tick
 * @param current Current encoder tick
 * @return wrapped difference
 * @note Encoder range 0~8191
 */
static  int32_t encoder_wrap_diff(int32_t target, int32_t current)
{
    int32_t diff = target - current;
    if (diff > 4096)       diff -= 8192;
    else if (diff < -4096) diff += 8192;
    return diff;
}

/**
 * @brief Compute unwrapped target position from current tick, use shortest path
 * @param current Current encoder tick
 * @param target Raw target encoder tick(0~8191)
 * @return Unwrapped absolute target position
 */
int32_t encoder_wrap_target(int32_t current, int32_t target)
{
    return current + encoder_wrap_diff(target, current);
}

