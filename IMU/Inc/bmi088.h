#pragma once
#ifndef BMI088_H
#define BMI088_H

#include "main.h" 

/* SPI句柄 */
extern SPI_HandleTypeDef hspi1;
#define BMI088_SPI hspi1

/* 传感器CS引脚定义 */
#define BMI088_ACCEL_CS_L() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define BMI088_ACCEL_CS_H() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define BMI088_GYRO_CS_L()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET)
#define BMI088_GYRO_CS_H()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET)

/* BMI088 ????????????????  */
#define BMI088_GYRO_CHIP_ID         0x00U
#define BMI088_GYRO_DATA             0x02U
#define BMI088_GYRO_RANGE            0x0FU
#define BMI088_GYRO_BANDWIDTH        0x10U
#define BMI088_GYRO_LPM1             0x11U
#define BMI088_GYRO_SOFTRESET        0x14U

/* BMI088 ????????????????  */
#define BMI088_ACCEL_CHIP_ID         0x00U
#define BMI088_ACCEL_DATA             0x12U
#define BMI088_ACCEL_CONF             0x40U
#define BMI088_ACCEL_RANGE            0x41U
#define BMI088_ACCEL_PWR_CTRL         0x7DU
#define BMI088_ACCEL_SOFTRESET        0x7EU

/* BMI088 ID ? */
#define BMI088_ACCEL_CHIP_ID_VALUE    0x1EU
#define BMI088_GYRO_CHIP_ID_VALUE     0x0FU

/* BMI088 传感器地址 */
#define BMI088_ACCEL_TEMP_L    0x22U  // 温度传感器低字节地址
#define BMI088_ACCEL_TEMP_H    0x23U  // 温度传感器高字节地址

/* 传感器参数 */
#define BMI088_ACCEL_SEN            0.0008974358974f  // 3g
#define BMI088_GYRO_SEN             0.0010652644f    // 2000°/s
#define BMI088_TEMP_FACTOR          0.125f        // 温度传感器系数/LSB
#define BMI088_TEMP_OFFSET          23.0f         // 温度传感器偏移量
/* IMU数据结构体 */
typedef struct {
    float accel[3];         // 加速度数据
    float gyro[3];          // 陀螺仪数据 (°/s)

    float gyro_offset[3];   // 陀螺仪零偏移
    float yaw_angle_deg;    // 俯仰角 (°)
    float yaw_rad;          // 俯仰角 (rad)
    
     float temp_init;            // 初始化温度
    float temp_comp_coeff;      // 温度补偿系数
    float temp_current;
    float gyro_cal;
    float temperature;      // 温度
} imu_t;

extern imu_t imu_data;

/* 初始化 */
uint8_t BMI088_Init(void);
//void BMI088_GyroRead(float gyro[3]);
//void BMI088_AccelRead(float accel[3]);
//float BMI088_ReadTemperature(void);
//void BMI088_Calibrate_Gyro(void);
void BMI088_ReadAll(float gyro[3], float accel[3], float temperature);
void BMI088_GyroRead(float gyro[3]);
void BMI088_AccelRead(float accel[3]);
float BMI088_ReadTemperature(void);
void BMI088_Calibrate_Gyro(void);
//void BMI088_Update(float dt); // 更新IMU数据，dt为时间间隔

#endif

