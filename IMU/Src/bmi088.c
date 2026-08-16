#include "bmi088.h"
#include <math.h>

imu_t imu_data = {0};

// SPI单字节收发底层函数
static uint8_t SPI_TransmitReceive(uint8_t tx_data)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_SPI, &tx_data, &rx_data, 1, 10);
    return rx_data;
}

// 读取BMI088指定寄存器
static uint8_t BMI088_ReadReg(uint8_t reg, uint8_t cs_pin)
{
    uint8_t data;
    if (cs_pin == 0) BMI088_ACCEL_CS_L();
    else BMI088_GYRO_CS_L();

    SPI_TransmitReceive(reg | 0x80); // 寄存器地址最高位置1，代表读操作
    data = SPI_TransmitReceive(0xFF); // 发送0xFF，同时接收寄存器返回数据

    if (cs_pin == 0) BMI088_ACCEL_CS_H();
    else BMI088_GYRO_CS_H();
    return data;
}

// 写入BMI088指定寄存器
static void BMI088_WriteReg(uint8_t reg, uint8_t data, uint8_t cs_pin)
{
    if (cs_pin == 0) BMI088_ACCEL_CS_L();
    else BMI088_GYRO_CS_L();

    SPI_TransmitReceive(reg & 0x7F); // 寄存器地址最高位置0，代表写操作
    SPI_TransmitReceive(data);       // 写入目标寄存器数据

    if (cs_pin == 0) BMI088_ACCEL_CS_H();
    else BMI088_GYRO_CS_H();
}

/**
 * @brief 加速度传感器初始化
 * @retval 0:初始化成功, 1:初始化失败
 */
static uint8_t BMI088_AccelInit(void)
{
    uint8_t id = 0;
/* BMI088加速度芯片默认上电为I2C模式，需要一次虚拟读操作切换为SPI模式 */
    BMI088_ReadReg(BMI088_ACCEL_CHIP_ID, 0); 
    HAL_Delay(1);

    /* 读取加速度芯片ID，校验通讯是否正常 */
    id = BMI088_ReadReg(BMI088_ACCEL_CHIP_ID, 0);
    if (id != BMI088_ACCEL_CHIP_ID_VALUE) {
        return 1;
    }

    /* 执行加速度传感器软件复位 */
    BMI088_WriteReg(BMI088_ACCEL_SOFTRESET, 0xB6, 0);
    HAL_Delay(50); // 复位操作需要等待稳定 

    /* 配置加速度传感器为工作模式 */
    BMI088_WriteReg(BMI088_ACCEL_PWR_CTRL, 0x04, 0); // 0x04 使能加速度传感器
    HAL_Delay(5); // 等待模式切换完成 

    /*配置加速度量程与输出频率*/
    BMI088_WriteReg(BMI088_ACCEL_RANGE, 0x00, 0); // 0x00 = ±3g量程
    BMI088_WriteReg(BMI088_ACCEL_CONF, 0x0A, 0);  // 0x0A = 100Hz输出，普通滤波模式

    HAL_Delay(10);
    return 0;
}

/**
 * @brief 陀螺仪传感器初始化
 * @retval 0:初始化成功, 1:初始化失败
 */
static uint8_t BMI088_GyroInit(void)
{
    uint8_t id = 0;
    
    
    /* 读取陀螺仪芯片ID，校验通讯是否正常 */
    id = BMI088_ReadReg(BMI088_GYRO_CHIP_ID, 1);
    if (id != BMI088_GYRO_CHIP_ID_VALUE) {
        return 1;
    }

    /* 执行陀螺仪软件复位 */
    BMI088_WriteReg(BMI088_GYRO_SOFTRESET, 0xB6, 1);
    HAL_Delay(20);

    /* 设置陀螺仪为正常工作模式 */
    BMI088_WriteReg(BMI088_GYRO_LPM1, 0x00, 1); // 正常工作模式，非低功耗

    /* 配置陀螺仪量程与输出带宽 */
    BMI088_WriteReg(BMI088_GYRO_RANGE, 0x00, 1);      // 0x00 = ±2000°/s量程
    BMI088_WriteReg(BMI088_GYRO_BANDWIDTH, 0x02, 1);  // 0x02 = 116Hz输出带宽

    HAL_Delay(10);
    return 0;
}

/**
 * @brief BMI088整体初始化入口函数
 * @retval 0:初始化成功, 1:初始化失败
 */
uint8_t BMI088_Init(void)
{
    if (BMI088_AccelInit() != 0) return 1;
    if (BMI088_GyroInit() != 0) return 1;
    return 0;
}

/**
 * @brief 读取三轴陀螺仪原始数据并转换为物理量
 * @param  gyro: 浮点数组，存储三轴角速度，单位：度/秒
 */
void BMI088_GyroRead(float gyro[3])
{
    uint8_t buf[6];
    int16_t raw[3];

    BMI088_GYRO_CS_L();

    /* 发送陀螺仪数据起始寄存器地址(0x02)，连续读取6字节三轴数据 */
    HAL_SPI_Transmit(&BMI088_SPI, (uint8_t*)"\x82", 1,10); // 读地址0x02 | 0x80读标志位
    HAL_SPI_Receive(&BMI088_SPI, buf, 6, 10);

    BMI088_GYRO_CS_H();

    /* 拼接16位原始数据 (BMI088数据格式为小端模式) */
    raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    /* 转换为实际角速度物理量(±2000°/s)，16位满量程换算系数：2000 / 32768 = 0.061035 */
    gyro[0] = raw[0] * 0.061035f;
    gyro[1] = raw[1] * 0.061035f;
    gyro[2] = raw[2] * 0.061035f;
}

/**
 * @brief 读取三轴加速度原始数据并转换为物理量
 * @param  accel: 浮点数组，存储三轴加速度，单位：g
 */
void BMI088_AccelRead(float accel[3])
{
    uint8_t buf[7];
    int16_t raw[3];

    BMI088_ACCEL_CS_L();

    /* 发送加速度数据起始寄存器地址(0x12)，连续读取6字节三轴数据 */
    HAL_SPI_Transmit(&BMI088_SPI, (uint8_t*)"\x92", 1,10); // 读地址0x12 | 0x80读标志位
    HAL_SPI_Receive(&BMI088_SPI, buf, 7, 10);

    BMI088_ACCEL_CS_H();

    /* 拼接16位原始数据 */
    raw[0] = (int16_t)((buf[2] << 8) | buf[1]);
    raw[1] = (int16_t)((buf[4] << 8) | buf[3]);
    raw[2] = (int16_t)((buf[6] << 8) | buf[5]);


    /* 转换为实际加速度物理量(±3g)，16位满量程换算系数：3 / 32768 = 0.000091552 */
    accel[0] = raw[0] * 0.000091552f;
    accel[1] = raw[1] * 0.000091552f;
    accel[2] = raw[2] * 0.000091552f;

}

///**
// * @brief 读取芯片内部温度，单位摄氏度
// * @retval 温度浮点值，单位℃
// */
// float BMI088_ReadTemperature(void)
//{
//    uint8_t temp_l, temp_h;
//    int16_t temp_raw;

//    /* 读取低字节 (寄存器地址 0x20) */
//    temp_l = BMI088_ReadReg(BMI088_GYRO_TEMP_X_L, 1);  // 1 = 陀螺仪片选
//    /* 读取高字节 (寄存器地址 0x21) */
//    temp_h = BMI088_ReadReg(BMI088_GYRO_TEMP_X_H, 1);

//    /* 小端拼接，高字节左移后或上低字节 */
//    temp_raw = (int16_t)((temp_h << 8) | temp_l);

//    /* BMI088温度换算公式(摄氏度) = 原始值 * 0.01 + 23.0 */
//    return (float)temp_raw * 0.01f + 23.0f;
//}

/**
 * @brief 批量读取连续寄存器数据
 * @param  reg:     读取起始寄存器地址
 * @param  buf:     数据存储缓冲区指针
 * @param  len:     需要读取的字节长度
 * @param  cs_pin:  片选通道 (0: 加速度芯片, 1: 陀螺仪芯片)
 */
static void BMI088_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len, uint8_t cs_pin)
{
    if (cs_pin == 0) BMI088_ACCEL_CS_L();
    else BMI088_GYRO_CS_L();

    /* 发送读寄存器地址，最高位自动置1 */
    HAL_SPI_Transmit(&BMI088_SPI, (uint8_t[]){reg | 0x80}, 1, 10);
    /* 批量读取 len 个字节数据 */
    HAL_SPI_Receive(&BMI088_SPI, buf, len, 10);

    if (cs_pin == 0) BMI088_ACCEL_CS_H();
    else BMI088_GYRO_CS_H();
}

/**
 * @brief 读取芯片内部温度，单位摄氏度
 * @retval 温度浮点值，单位℃
 */
float BMI088_ReadTemperature(void)
{
    uint8_t buf[2];
    uint16_t temp_uint11;
    int16_t temp_int11;

    // 读取温度寄存器高低字节
    BMI088_ReadRegs(BMI088_ACCEL_TEMP_L, buf, 2, 0);  // 0 = 加速度芯片片选

    // 拼接11位无符号原始温度值
    temp_uint11 = (buf[1] << 3) | (buf[0] >> 5);

    // 转换为有符号11位数值（正负温度处理）
    if (temp_uint11 > 1023) {
        temp_int11 = (int16_t)(temp_uint11 - 2048);
    } else {
        temp_int11 = (int16_t)temp_uint11;
    }

    // 换算为实际摄氏度温度
    return (float)temp_int11 * 0.125f + 23.0f;
}

/**
 * @brief 陀螺仪静态校准，采集1000次数据计算Z轴零偏
 */
void BMI088_Calibrate_Gyro(void) {
    float sum = 0;
    float temp_gyro[3];
    for (int i = 0; i < 1000; i++) {
        BMI088_GyroRead(temp_gyro);
        sum += temp_gyro[2]; // 仅校准Z轴角速度零偏
        HAL_Delay(1);
    }
    imu_data.gyro_offset[2] = sum / 1000.0f; // 存储平均零偏值
}

///**
// * @brief 1ms周期IMU更新函数，仅解算Yaw航向角
// */
//void BMI088_Update(float dt) {
//
//    BMI088_AccelRead(imu_data.accel);
//    BMI088_GyroRead(imu_data.gyro);
//
//    // 减去Z轴零偏，增加死区过滤微小抖动
//    float gz = imu_data.gyro[2] - imu_data.gyro_offset[2];
//    if (fabsf(gz) < 0.05f) gz = 0.0f;
//
//    // 积分求解航向角：角度 = 原角度 + 角速度 * 时间步长
//    imu_data.yaw_angle_deg += gz * dt;
//    // 角度转为弧度存储
//    imu_data.yaw_rad = imu_data.yaw_angle_deg * (3.14159265f / 180.0f);
//}
