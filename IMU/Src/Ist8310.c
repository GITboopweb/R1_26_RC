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

    // 复位脉冲（低→高）
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(10);

    // 验证 ID
    if (IST8310_ReadReg(IST8310_WHO_AM_I, &id) != HAL_OK || id != IST8310_WHO_AM_I_VAL)
        return 1;

    // 软复位（CTRL1 bit2=1）
    if (IST8310_WriteReg(IST8310_CTRL1, 0x0D) != HAL_OK) return 1;
    HAL_Delay(10);

    // 配置中断、采样次数、固定值、输出速率（参照文档）
    IST8310_WriteReg(0x0B, 0x01);    // 中断使能，低电平有效
    IST8310_WriteReg(0x41, 0x12);    // X/Y/Z 各 2 次采样
    IST8310_WriteReg(0x42, 0xC0);    // 固定必须值
    IST8310_WriteReg(0x0A, 0x0C);    // 200Hz（若手册不同请核对）

    // 进入普通模式（CTRL1 bit0=1）
    if (IST8310_WriteReg(IST8310_CTRL1, 0x01) != HAL_OK) return 1;
    HAL_Delay(5);

    return 0;
}

uint8_t IST8310_ReadMag(float mag[3])
{
    uint8_t status;
    uint8_t buf[6];
    int16_t raw[3];

    // 检查数据就绪
    if (IST8310_ReadReg(IST8310_STATUS, &status) != HAL_OK) return 1;
    if (!(status & 0x01)) return 1;   // 无新数据

    // 读取 6 个字节
    if (HAL_I2C_Mem_Read(&hi2c3, IST8310_DEVICE_ADDR, IST8310_DATA_X_L,
                         I2C_MEMADD_SIZE_8BIT, buf, 6, 10) != HAL_OK)
        return 1;

    raw[0] = (int16_t)((buf[1] << 8) | buf[0]);
    raw[1] = (int16_t)((buf[3] << 8) | buf[2]);
    raw[2] = (int16_t)((buf[5] << 8) | buf[4]);

    mag[0] = raw[0] * IST8310_SENSITIVITY;
    mag[1] = raw[1] * IST8310_SENSITIVITY;
    mag[2] = raw[2] * IST8310_SENSITIVITY;

    return 0;
}

/**
 * @brief 根据磁力计计算航向角（仅水平时准确）
 * @retval 航向角（度，范围 -180 ~ 180）
 */
float IST8310_GetHeading(const float mag[3])
{
    float heading = atan2f(mag[1], mag[0]) * 180.0f / 3.14159265f;
    return IST8310_WrapAngleDeg(heading);
}
