#include "mpu6050.h"
#include "i2c.h"

/* ===================== DMA 模式变量 ===================== */

static MPU6050_Mode mpu6050_mode = MPU6050_MODE_BLOCKING;  // 默认阻塞模式
static uint8_t mpu6050_dma_buffer[14];  // DMA 接收缓冲区
static volatile uint8_t mpu6050_dma_ready = 0;  // DMA 完成标志

/**
 * @brief  MPU6050写寄存器
 * @param  RegAddress 寄存器地址
 * @param  Data 要写入的数据
 * @retval HAL状态
 */
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    return HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDRESS, RegAddress,
                             I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
}

/**
 * @brief  MPU6050读寄存器
 * @param  RegAddress 寄存器地址
 * @retval 读取到的数据
 */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS, RegAddress,
                     I2C_MEMADD_SIZE_8BIT, &Data, 1, HAL_MAX_DELAY);
    return Data;
}

/**
 * @brief  MPU6050连续读多个寄存器
 * @param  RegAddress 起始寄存器地址
 * @param  DataArray 数据存储数组
 * @param  Count 读取字节数
 * @retval HAL状态
 */
HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *DataArray, uint8_t Count)
{
    return HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS, RegAddress,
                            I2C_MEMADD_SIZE_8BIT, DataArray, Count, HAL_MAX_DELAY);
}

/**
 * @brief  MPU6050初始化
 * @param  无
 * @retval HAL状态
 */
HAL_StatusTypeDef MPU6050_Init(void)
{
    HAL_StatusTypeDef status;

    /* 等待MPU6050上电稳定 */
    HAL_Delay(100);

    /* 检查设备ID */
    uint8_t id = MPU6050_GetID();
    if (id != 0x68) {
        return HAL_ERROR;  // 设备ID不匹配
    }

    /* 电源管理寄存器1: 解除睡眠,选择X轴陀螺仪作为时钟源 */
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    /* 电源管理寄存器2: 所有轴均不待机 */
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    if (status != HAL_OK) return status;

    /* 采样率分频: 1kHz / (1 + 9) = 100Hz */
    status = MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    if (status != HAL_OK) return status;

    /* 配置寄存器: DLPF = 1 (188Hz带宽, 2ms延迟)
     * 关闭内部低通滤波，避免干扰 EKF 算法
     * DLPF=0: 256Hz (无延迟, 但噪声大)
     * DLPF=1: 188Hz (推荐, 平衡噪声和延迟)
     * DLPF=2: 98Hz  (适中)
     */
    status = MPU6050_WriteReg(MPU6050_CONFIG, 0x01);
    if (status != HAL_OK) return status;

    /* 陀螺仪配置: 满量程 ±2000°/s */
    status = MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    if (status != HAL_OK) return status;

    /* 加速度计配置: 满量程 ±16g */
    status = MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

/**
 * @brief  获取MPU6050设备ID
 * @param  无
 * @retval 设备ID (正常应为0x68)
 */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
 * @brief  获取MPU6050加速度和陀螺仪数据
 * @param  AccX, AccY, AccZ 加速度计X/Y/Z轴数据 (±16g量程)
 * @param  GyroX, GyroY, GyroZ 陀螺仪X/Y/Z轴数据 (±2000°/s量程)
 * @retval HAL状态
 */
HAL_StatusTypeDef MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                                   int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t Data[14];
    HAL_StatusTypeDef status;

    /* 从0x3B开始连续读取14字节 */
    status = MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, Data, 14);
    if (status != HAL_OK) return status;

    /* 数据拼接 (大端序) */
    *AccX = (int16_t)((Data[0] << 8) | Data[1]);
    *AccY = (int16_t)((Data[2] << 8) | Data[3]);
    *AccZ = (int16_t)((Data[4] << 8) | Data[5]);
    // Data[6-7] 为温度数据,此处不使用
    *GyroX = (int16_t)((Data[8] << 8) | Data[9]);
    *GyroY = (int16_t)((Data[10] << 8) | Data[11]);
    *GyroZ = (int16_t)((Data[12] << 8) | Data[13]);

    return HAL_OK;
}

/* ===================== DMA 模式函数 ===================== */

/**
 * @brief  设置 MPU6050 读取模式
 * @param  mode: MPU6050_MODE_BLOCKING (阻塞) 或 MPU6050_MODE_DMA
 * @retval 无
 */
void MPU6050_SetMode(MPU6050_Mode mode)
{
    mpu6050_mode = mode;
}

/**
 * @brief  启动 DMA 读取 (非阻塞)
 * @param  无
 * @retval HAL 状态
 * @note   读取完成后会触发 HAL_I2C_MemRxCpltCallback
 */
HAL_StatusTypeDef MPU6050_StartDMA_Read(void)
{
    mpu6050_dma_ready = 0;
    return HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
                                 I2C_MEMADD_SIZE_8BIT, mpu6050_dma_buffer, 14);
}

/**
 * @brief  检查 DMA 读取是否完成
 * @param  无
 * @retval 1=完成, 0=未完成
 */
uint8_t MPU6050_IsDMA_Ready(void)
{
    return mpu6050_dma_ready;
}

/**
 * @brief  从 DMA 缓冲区获取数据
 * @param  AccX, AccY, AccZ, GyroX, GyroY, GyroZ
 * @retval HAL 状态
 * @note   调用前需确保 MPU6050_IsDMA_Ready() 返回 1
 */
HAL_StatusTypeDef MPU6050_GetData_DMA(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                                       int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    if (!mpu6050_dma_ready) {
        return HAL_BUSY;  // DMA 未完成
    }

    /* 从 DMA 缓冲区解析数据 */
    *AccX = (int16_t)((mpu6050_dma_buffer[0] << 8) | mpu6050_dma_buffer[1]);
    *AccY = (int16_t)((mpu6050_dma_buffer[2] << 8) | mpu6050_dma_buffer[3]);
    *AccZ = (int16_t)((mpu6050_dma_buffer[4] << 8) | mpu6050_dma_buffer[5]);
    *GyroX = (int16_t)((mpu6050_dma_buffer[8] << 8) | mpu6050_dma_buffer[9]);
    *GyroY = (int16_t)((mpu6050_dma_buffer[10] << 8) | mpu6050_dma_buffer[11]);
    *GyroZ = (int16_t)((mpu6050_dma_buffer[12] << 8) | mpu6050_dma_buffer[13]);

    mpu6050_dma_ready = 0;  // 清除标志
    return HAL_OK;
}

/**
 * @brief  I2C DMA 接收完成回调
 * @param  hi2c: I2C 句柄
 * @retval 无
 * @note   此函数会被 HAL 库自动调用
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2) {
        mpu6050_dma_ready = 1;  // 标记 DMA 完成
    }
}

/* ===================== IMU_Read_Raw (支持 DMA 模式) ===================== */

/**
 * @brief  IMU_Read_Raw - 为 EKF/INS 提供原始数据接口
 * @param  gyro[3] 陀螺仪原始值 (LSB, 需转换为 rad/s)
 * @param  acc[3]  加速度计原始值 (LSB, 需转换为 g)
 * @note   此函数由 ekf.c 和 ins.c 调用
 *         支持阻塞模式和 DMA 模式 (通过 MPU6050_SetMode 切换)
 */
void IMU_Read_Raw(float gyro[3], float acc[3])
{
    int16_t ax, ay, az, gx, gy, gz;
    HAL_StatusTypeDef status;

    if (mpu6050_mode == MPU6050_MODE_DMA) {
        /* DMA 模式: 从缓冲区读取 */
        status = MPU6050_GetData_DMA(&ax, &ay, &az, &gx, &gy, &gz);

        /* 启动下一次 DMA 读取 */
        if (status == HAL_OK) {
            MPU6050_StartDMA_Read();
        }
    } else {
        /* 阻塞模式: 直接读取 */
        status = MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
    }

    if (status == HAL_OK) {
        /* 陀螺仪: 保持 LSB 原始值 (由 EKF 内部转换为 rad/s)
         * ±2000°/s 量程, 灵敏度 16.384 LSB/(°/s)
         */
        gyro[0] = (float)gx;
        gyro[1] = (float)gy;
        gyro[2] = (float)gz;

        /* 加速度计: LSB → g
         * ±16g 量程, 灵敏度 2048 LSB/g
         */
        acc[0] = (float)ax / 2048.0f;
        acc[1] = (float)ay / 2048.0f;
        acc[2] = (float)az / 2048.0f;
    } else {
        /* 读取失败, 返回零值 */
        gyro[0] = gyro[1] = gyro[2] = 0.0f;
        acc[0] = acc[1] = acc[2] = 0.0f;
    }
}
