/*
 * ekf.c - 扩展卡尔曼滤波器 (EKF) 实现
 *
 * 参考: 逐飞 CYT4BB 开源库 ekf.c
 *
 * 状态向量: 四元数 X = [q0, q1, q2, q3]ᵀ
 * 观测向量: 归一化加速度 Z = [ax, ay, az]ᵀ
 *
 * EKF 流程:
 *   1. 读取 IMU 数据 (陀螺仪 + 加速度计)
 *   2. 构建状态转移矩阵 F
 *   3. 预测: X_ = F×X, P_ = F×P×Fᵀ + Q
 *   4. 观测更新: K = P_×Hᵀ×(H×P_×Hᵀ+R)⁻¹
 *   5. 状态更新: X = X_ + K×(Z - h(X_))
 *   6. 协方差更新: P = (I - K×H)×P_
 *   7. 四元数转欧拉角
 *
 * 重构说明:
 *   EKF_UpData 和 EKF_Update_NoRead 的核心算法完全相同,
 *   仅 IMU 数据读取方式不同。故提取为 EKF_Update_Core(),
 *   两个入口函数分别负责调用。
 */

#include "ekf.h"
#include <stdio.h>

/* ===================== EKF 内部变量 ===================== */

static Matrix4 Q_mat;   // 过程噪声协方差矩阵 (4×4)
static Matrix3 R_mat;  // 观测噪声协方差矩阵 (3×3)
static Matrix4 P_mat;  // 误差协方差矩阵 (4×4)

static float acc_x_l = 0.0f;  // 加速度计低通滤波历史值
static float acc_y_l = 0.0f;
static float acc_z_l = 0.0f;

/* 陀螺仪零漂校准值 (原始LSB)
 * 参考教程 02-俯仰角与互补滤波 中的 GY -= 16
 * 不同芯片零漂不同，需要实测校准
 *
 * 调试选项: USE_FIXED_GYRO_OFFSET = 1 使用固定初始值，= 0 使用校准函数
 * 教程参考值: Y轴零漂约 16 */
#define USE_FIXED_GYRO_OFFSET 0

#if USE_FIXED_GYRO_OFFSET
/* 使用固定零漂值 (教程参考值 16) */
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 16.0f;   // 教程参考值: 16
static float gyro_z_offset = 0.0f;
#else
/* 使用校准函数设置的零漂值 */
static float gyro_x_offset = 0.0f;   // X轴零漂 (横滚轴)
static float gyro_y_offset = 0.0f;   // Y轴零漂 (俯仰轴)
static float gyro_z_offset = 0.0f;   // Z轴零漂 (偏航轴)
#endif

/* ===================== 全局变量定义 ===================== */

Vector4 ekf_state;          // 状态向量: 四元数
EulerAngles euler_angle;    // 输出: 欧拉角
imu_t imu_data = {0};       // IMU 数据
float ekf_mahalDist = 0.0f; // 马氏距离

/* INS 陀螺仪零偏 (由 ins.c 使用) */
GyroOffset gyro_offset = {0, 0, 0};

/* ===================== 噪声矩阵常量 ===================== */

/* 过程噪声 Q (对角阵, 值越大对陀螺仪信任度越低) */
static const float Q_data[4][4] = {
    {Q_DIAG, 0,     0,     0    },
    {0,     Q_DIAG, 0,     0    },
    {0,     0,     Q_DIAG, 0    },
    {0,     0,     0,     Q_DIAG}
};

/* 观测噪声 R (对角阵, 值越大对加速度计信任度越低) */
static const float R_data[3][3] = {
    {R_DIAG, 0,     0     },
    {0,     R_DIAG, 0     },
    {0,     0,     R_DIAG}
};

/* 初始误差协方差 P (大值表示初始不确定) */
static const float P_init[4][4] = {
    {P_DIAG, 0,       0,       0      },
    {0,       P_DIAG, 0,       0      },
    {0,       0,       P_DIAG, 0      },
    {0,       0,       0,       P_DIAG}
};

/* ===================== 内部函数 ===================== */

/*
 * Quaternion_To_Euler - 四元数转欧拉角
 *
 * 从四元数计算 pitch/roll/yaw, 存入全局 euler_angle
 */
static void Quaternion_To_Euler(void)
{
    float q0 = ekf_state.data[0];
    float q1 = ekf_state.data[1];
    float q2 = ekf_state.data[2];
    float q3 = ekf_state.data[3];

    /* 俯仰角 (pitch): asin(-2*q1*q3 + 2*q0*q2) */
    euler_angle.pitch = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * RAD_TO_DEG;

    /* 横滚角 (roll): atan2(...) */
    euler_angle.roll = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1,
                              -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * RAD_TO_DEG;

    /* 偏航角 (yaw): atan2(...) */
    euler_angle.yaw = atan2(2.0f * q1 * q2 + 2.0f * q0 * q3,
                             -2.0f * q2 * q2 - 2.0f * q3 * q3 + 1.0f) * RAD_TO_DEG;
}

/*
 * IMU_Get_Values - 读取并处理 IMU 数据
 *
 * 1. 调用 IMU_Read_Raw() 获取原始数据
 * 2. 加速度计低通滤波
 * 3. 陀螺仪 LSB 转 rad/s
 */
static void IMU_Get_Values(void)
{
    float gyro_raw[3];
    float acc_raw[3];

    /* 读取原始数据 */
    IMU_Read_Raw(gyro_raw, acc_raw);

    /* Z轴取反: 使Z+朝上，符合右手坐标系 (静止时acc_z≈+g) */
    float acc_z_neg = -acc_raw[2];

    /* 加速度计低通滤波 (X/Y/Z 三轴) */
    imu_data.acc_x = ACC_LPF_K * acc_raw[0]  + (1.0f - ACC_LPF_K) * acc_x_l;
    imu_data.acc_y = ACC_LPF_K * acc_raw[1]  + (1.0f - ACC_LPF_K) * acc_y_l;
    imu_data.acc_z = ACC_LPF_K * acc_z_neg   + (1.0f - ACC_LPF_K) * acc_z_l;

    /* 保存本次滤波值，供下次滤波使用 */
    acc_x_l = imu_data.acc_x;
    acc_y_l = imu_data.acc_y;
    acc_z_l = imu_data.acc_z;

    /* 陀螺仪原始值转 rad/s (根据 MPU6050 量程 ±2000°/s, 灵敏度 16.384 LSB/(°/s))
     * 应用零漂校准
     * Z轴取反：与加速度计Z轴方向一致
     */
    float gyro_lsb_to_rads = PI / 180.0f / 16.384f;
    imu_data.gyro_x = (gyro_raw[0] - gyro_x_offset) * gyro_lsb_to_rads;
    imu_data.gyro_y = (gyro_raw[1] - gyro_y_offset) * gyro_lsb_to_rads;
    imu_data.gyro_z = -(gyro_raw[2] - gyro_z_offset) * gyro_lsb_to_rads;
}

/*
 * EKF_Update_Core - EKF 核心算法 (内部使用)
 *
 * EKF_UpData 和 EKF_Update_NoRead 的公共实现部分:
 *   读取 imu_data → 构建 F/H → 预测 → 观测更新 → 输出欧拉角
 *
 * @return 0: 正常更新
 *         1: 矩阵奇异跳过
 *         2: 马氏距离过大跳过
 */
static uint8_t EKF_Update_Core(void)
{
    float gx = imu_data.gyro_x;
    float gy = imu_data.gyro_y;
    float gz = imu_data.gyro_z;
    float dt = EKF_DT;

    /* ===== 第1步: 构建观测向量 Z (归一化加速度) ===== */
    Vector3 Z;
    Z.data[0] = imu_data.acc_x;
    Z.data[1] = imu_data.acc_y;
    Z.data[2] = imu_data.acc_z;
    Vector3_Normalize(&Z);

    /* ===== 第2步: 构建状态转移矩阵 F ===== */
    Matrix4 F;
    float half_dt = 0.5f * dt;
    float f_data[4][4] = {
        { 1,               -half_dt*gx, -half_dt*gy, -half_dt*gz },
        { half_dt*gx,      1,            half_dt*gz, -half_dt*gy },
        { half_dt*gy,     -half_dt*gz,  1,            half_dt*gx },
        { half_dt*gz,      half_dt*gy, -half_dt*gx,  1          }
    };
    Matrix_From_Array4(&F, (const float*)f_data, 4, 4);
    Matrix4 FT = Matrix_Transpose4(&F);

    /* ===== 第3步: 预测状态 X_ = F × X ===== */
    Vector4 X_pred;
    for (int i = 0; i < 4; i++) {
        X_pred.data[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            X_pred.data[i] += F.data[i][j] * ekf_state.data[j];
        }
    }
    Vector4_Normalize(&X_pred);

    /* ===== 第4步: 预测协方差 P_ = F × P × Fᵀ + Q ===== */
    Matrix4 FP   = Matrix_Multiply4(&F, &P_mat);
    Matrix4 FPFt = Matrix_Multiply4(&FP, &FT);
    Matrix4 P_pred = Matrix_Add4(&FPFt, &Q_mat);

    /* ===== 第5步: 构建观测矩阵 H (重力向量对四元数的雅可比, 3×4) ===== */
    float q0 = X_pred.data[0];
    float q1 = X_pred.data[1];
    float q2 = X_pred.data[2];
    float q3 = X_pred.data[3];

    /* H = 2 × h(q), 其中 h(q) 是重力向量的四元数表达式:
     *   h₀ = 2(q₁q₃ - q₀q₂)
     *   h₁ = 2(q₀q₁ + q₂q₃)
     *   h₂ = q₀² - q₁² - q₂² + q₃²
     *
     * 雅可比矩阵 ∂h/∂q:
     *   [-2q₂,  2q₃, -2q₀,  2q₁]
     *   [ 2q₁,  2q₀,  2q₃,  2q₂]
     *   [ 2q₀, -2q₁, -2q₂,  2q₃]
     */
    float h_data[3][4] = {
        { -2.0f*q2,  2.0f*q3, -2.0f*q0,  2.0f*q1 },
        {  2.0f*q1,  2.0f*q0,  2.0f*q3,  2.0f*q2 },
        {  2.0f*q0, -2.0f*q1, -2.0f*q2,  2.0f*q3 }
    };
    Matrix4 H_mat;
    Matrix_From_Array4(&H_mat, (const float*)h_data, 3, 4);
    Matrix4 HT_mat = Matrix_Transpose4(&H_mat);

    /* ===== 第6步: 计算新息协方差 D = H × P_ × Hᵀ + R ===== */
    /* D = H×P_×Hᵀ + R (3×3), 用 float 局部数组避免 Matrix4 框架开销 */
    float HP[3][4];
    float HPHT[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += H_mat.data[i][k] * P_pred.data[k][j];
            }
            HP[i][j] = s;
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += HP[i][k] * HT_mat.data[k][j];
            }
            HPHT[i][j] = s + (i == j ? R_data[i][i] : 0.0f);
        }
    }

    /* D⁻¹ */
    float invD[3][3];
    if (Matrix3_Inverse(HPHT, invD)) {
        /* 矩阵奇异, 跳过观测更新，保留预测结果 */
        ekf_state = X_pred;
        P_mat = P_pred;
        Quaternion_To_Euler();
        return 1;
    }

    /* ===== 第7步: 计算新息 EK = Z - h(X_) ===== */
    float EK[3];
    EK[0] = Z.data[0] - (2.0f * (q1 * q3 - q0 * q2));
    EK[1] = Z.data[1] - (2.0f * (q0 * q1 + q2 * q3));
    EK[2] = Z.data[2] - (1.0f - 2.0f * q1 * q1 - 2.0f * q2 * q2);

    /* ===== 第8步: 马氏距离检验 r = EKᵀ × D⁻¹ × EK ===== */
    float r = 0.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r += EK[i] * invD[i][j] * EK[j];
        }
    }
    ekf_mahalDist = r;

    if (r > MAHALANOBIS_THRESHOLD) {
        /* 保留预测结果 (协方差已含 Q 的增长) */
        ekf_state = X_pred;
        P_mat = P_pred;
        Quaternion_To_Euler();
        return 2;
    }

    /* ===== 第9步: 计算卡尔曼增益 K = P_ × Hᵀ × D⁻¹ (4×3) ===== */
    /* 完整: K = P_ × Hᵀ × D⁻¹
     * 分步: S = P_ × Hᵀ (4×3), 再 K = S × D⁻¹ (4×3) */
    float S[4][3];  // S = P_ × Hᵀ
    for (int i = 0; i < 4; i++) {
        for (int m = 0; m < 3; m++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++) {
                s += P_pred.data[i][k] * H_mat.data[m][k];
            }
            S[i][m] = s;
        }
    }

    float K_data[4][3];  // K = S × D⁻¹
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int m = 0; m < 3; m++) {
                s += S[i][m] * invD[m][j];
            }
            K_data[i][j] = s;
        }
    }

    /* ===== 第10步: 状态更新 X = X_ + K × EK ===== */
    for (int i = 0; i < 4; i++) {
        float s = 0.0f;
        for (int j = 0; j < 3; j++) {
            s += K_data[i][j] * EK[j];
        }
        ekf_state.data[i] = X_pred.data[i] + s;
    }
    Vector4_Normalize(&ekf_state);

    /* ===== 第11步: 协方差更新 P = (I - K × H) × P_ ===== */
    Matrix4 KH;
    Matrix_Init4(&KH, 4, 4);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += K_data[i][k] * H_mat.data[k][j];
            }
            KH.data[i][j] = s;
        }
    }

    Matrix4 I4;
    Matrix_Identity4(&I4, 4);
    Matrix4 I_minus_KH = Matrix_Subtract4(&I4, &KH);
    Matrix4 P_new = Matrix_Multiply4(&I_minus_KH, &P_pred);
    P_mat = P_new;

    /* ===== 第12步: 四元数转欧拉角 ===== */
    Quaternion_To_Euler();

    return 0;
}

/* ===================== 外部函数 ===================== */

/*
 * EKF_Init - EKF 初始化
 *
 * 调用一次即可
 */
void EKF_Init(void)
{
    /* 初始化四元数状态 [q0, q1, q2, q3] = [1, 0, 0, 0] */
    ekf_state.data[0] = 1.0f;
    ekf_state.data[1] = 0.0f;
    ekf_state.data[2] = 0.0f;
    ekf_state.data[3] = 0.0f;

    /* 初始化噪声矩阵 */
    Matrix_From_Array4(&Q_mat, (const float*)Q_data, 4, 4);
    Matrix_From_Array3(&R_mat, (const float*)R_data, 3, 3);
    Matrix_From_Array4(&P_mat, (const float*)P_init, 4, 4);

    /* 初始化欧拉角 */
    euler_angle.pitch = 0.0f;
    euler_angle.roll  = 0.0f;
    euler_angle.yaw   = 0.0f;
}

/*
 * EKF_CalibrateGyro - 陀螺仪零漂校准
 *
 * 调用方法: 将MPU6050水平静止放置，调用此函数
 * 函数会自动采集样本，计算零漂值并保存
 *
 * @param samples  采样次数 (建议100-500)
 * @param delay_ms 每次采样间隔 (ms)
 */
void EKF_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    float gyro_raw[3], acc_raw[3];

    printf("开始陀螺仪校准，请保持传感器静止...\r\n");

    for (uint16_t i = 0; i < samples; i++) {
        IMU_Read_Raw(gyro_raw, acc_raw);
        sum_x += gyro_raw[0];
        sum_y += gyro_raw[1];
        sum_z += gyro_raw[2];
        HAL_Delay(delay_ms);
    }

    gyro_x_offset = sum_x / samples;
    gyro_y_offset = sum_y / samples;
    gyro_z_offset = sum_z / samples;

    printf("校准完成! 零漂值: X=%.1f Y=%.1f Z=%.1f\r\n",
           gyro_x_offset, gyro_y_offset, gyro_z_offset);
}

/*
 * EKF_UpData - EKF 一步更新 (定时器中断调用)
 *
 * 1. 读取 IMU 数据
 * 2. 调用 EKF 核心算法
 */
void EKF_UpData(void)
{
    IMU_Get_Values();
    EKF_Update_Core();
}

/*
 * EKF_GetAngles - 获取欧拉角
 */
const EulerAngles* EKF_GetAngles(void)
{
    return &euler_angle;
}

/* ===================== 主循环调用接口 (不依赖中断) ===================== */

/*
 * IMU_UpdateBuffer - 更新 IMU 数据缓冲区
 *
 * 由主循环调用，将读取到的原始 IMU 数据写入缓冲区
 * @param gyro_raw[3] 陀螺仪原始值 (LSB, ±2000°/s量程)
 * @param acc_raw[3]  加速度计原始值 (原始ADC)
 *
 * 与 IMU_Get_Values() 相同的处理逻辑，但不调用 IMU_Read_Raw()
 */
void IMU_UpdateBuffer(float gyro_raw[3], float acc_raw[3])
{
    /* Z轴取反: 使Z+朝上，符合右手坐标系 (静止时acc_z≈+g) */
    float acc_z_neg = -acc_raw[2];

    /* 加速度计低通滤波 (X/Y/Z 三轴) */
    imu_data.acc_x = ACC_LPF_K * acc_raw[0]  + (1.0f - ACC_LPF_K) * acc_x_l;
    imu_data.acc_y = ACC_LPF_K * acc_raw[1]  + (1.0f - ACC_LPF_K) * acc_y_l;
    imu_data.acc_z = ACC_LPF_K * acc_z_neg   + (1.0f - ACC_LPF_K) * acc_z_l;

    /* 保存本次滤波值，供下次滤波使用 */
    acc_x_l = imu_data.acc_x;
    acc_y_l = imu_data.acc_y;
    acc_z_l = imu_data.acc_z;

    /* 陀螺仪原始值转 rad/s (MPU6050 ±2000°/s, 灵敏度 16.384 LSB/(°/s))
     * 应用零漂校准, Z轴取反与加速度计Z轴方向一致
     */
    float gyro_lsb_to_rads = PI / 180.0f / 16.384f;
    imu_data.gyro_x = (gyro_raw[0] - gyro_x_offset) * gyro_lsb_to_rads;
    imu_data.gyro_y = (gyro_raw[1] - gyro_y_offset) * gyro_lsb_to_rads;
    imu_data.gyro_z = -(gyro_raw[2] - gyro_z_offset) * gyro_lsb_to_rads;
}

/*
 * EKF_Update_NoRead - EKF 更新 (不读取 IMU, 主循环调用)
 *
 * 使用预更新的 imu_data (由 IMU_UpdateBuffer 填充) 运行 EKF 核心算法
 * @return 0: 正常更新, 1: 矩阵奇异跳过, 2: 马氏距离过大跳过
 */
uint8_t EKF_Update_NoRead(void)
{
    return EKF_Update_Core();
}