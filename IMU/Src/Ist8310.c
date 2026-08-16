#include "ist8310.h"

static HAL_StatusTypeDef IST8310_WriteReg(uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(&hi2c3,IST8310_DEVICE_ADDR, reg,I2C_MEMADD_SIZE_8BIT, &data,1,10);
}

static HAL_StatusTypeDef IST8310_ReadReg(uint8_t reg, uint8_t* data)
{
    return HAL_I2C_Mem_Read(&hi2c3, IST8310_DEVICE_ADDR, reg,I2C_MEMADD_SIZE_8BIT, data,1,10);
}

static float IST8310_WrapAngleDeg(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)  angle -= 360.0f;
    if (angle < -180.0f) angle += 360.0f;
    return angle;
}

uint8_t IST8310_Init(void)
{
    uint8_t id = 0;

    // 复位磁力计电源引脚，硬件复位芯片
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(10);

    // 读取芯片ID校验通讯是否正常
    if (IST8310_ReadReg(IST8310_WHO_AM_I, &id) != HAL_OK || id != IST8310_WHO_AM_I_VAL)
        return 1;

    // 进入配置寄存器模式 CTRL1 bit2=1
    if (IST8310_WriteReg(IST8310_CTRL1, 0x0D) != HAL_OK) return 1;
    HAL_Delay(10);

    // 磁场补偿参数配置，优化地磁采样精度与噪声抑制
    IST8310_WriteReg(0x0B, 0x01);    // 启用内部硬磁补偿，改善平面精度
    IST8310_WriteReg(0x41, 0x12);    // X/Y/Z轴各配置2阶滤波
    IST8310_WriteReg(0x42, 0xC0);    // 设置磁场增益补偿参数
    IST8310_WriteReg(0x0A, 0x0C);    // 输出速率200Hz，连续采样模式

    // 切换为连续采集工作模式 CTRL1 bit0=1
    if (IST8310_WriteReg(IST8310_CTRL1, 0x01) != HAL_OK) return 1;
    HAL_Delay(5);

    return 0;
}

uint8_t IST8310_ReadMag(float mag[3])
{
    uint8_t status;
    uint8_t buf[6];
    int16_t raw[3];

    // 查询数据就绪标志位
    if (IST8310_ReadReg(IST8310_STATUS, &status) != HAL_OK) return 1;
    if (!(status & 0x01)) return 1;   // 数据未就绪，直接退出

    // 连续读取三轴地磁6字节原始数据
    if (HAL_I2C_Mem_Read(&hi2c3, IST8310_DEVICE_ADDR, IST8310_DATA_X_L,
                         I2C_MEMADD_SIZE_8BIT, buf, 6, 10) != HAL_OK)
        return 1;

    // 高低字节拼接为16位有符号原始采样值
    raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    // 乘以灵敏度系数转换为实际地磁物理量
    mag[0] = raw[0] * IST8310_SENSITIVITY;
    mag[1] = raw[1] * IST8310_SENSITIVITY;
    mag[2] = raw[2] * IST8310_SENSITIVITY;

    return 0;
}

/**
 * @brief 根据三轴地磁数据计算水平航向角（设备水平放置时有效）
 * @param mag 三轴地磁浮点数组
 * @retval 航向角度，范围 -180 ~ 180 度
 */
float IST8310_GetHeading(const float mag[3])
{
    // atan2(Y,X)计算地磁平面夹角并转为角度
    float heading = atan2f(mag[1], mag[0]) * 180.0f / 3.14159265f;
    // 角度限幅至[-180,180]区间
    return IST8310_WrapAngleDeg(heading);
}
