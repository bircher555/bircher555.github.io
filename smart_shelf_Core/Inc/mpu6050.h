#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* MPU6050 I2C地址 (AD0接地时为0x68, 接VCC时为0x69) */
#define MPU6050_ADDRESS     0xD0    // 0x68 << 1 (HAL库需要左移1位)

/* MPU6050寄存器地址 */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48

#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_PWR_MGMT_2      0x6C
#define MPU6050_WHO_AM_I        0x75

/* DMA 模式选择 */
typedef enum {
    MPU6050_MODE_BLOCKING = 0,  // 阻塞模式 (默认)
    MPU6050_MODE_DMA            // DMA 模式
} MPU6050_Mode;

/* 函数声明 */
HAL_StatusTypeDef MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
HAL_StatusTypeDef MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                                   int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);

/* DMA 模式函数 */
void MPU6050_SetMode(MPU6050_Mode mode);
HAL_StatusTypeDef MPU6050_StartDMA_Read(void);
uint8_t MPU6050_IsDMA_Ready(void);
HAL_StatusTypeDef MPU6050_GetData_DMA(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                                       int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
