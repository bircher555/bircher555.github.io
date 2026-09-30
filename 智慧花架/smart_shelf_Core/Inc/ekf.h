/*
 * ekf.h - 扩展卡尔曼滤波器 (EKF)
 *
 * 用于机器人姿态估计：融合陀螺仪（角速度）和加速度计数据，
 * 输出高精度的横滚角（roll）、俯仰角（pitch）、偏航角（yaw）。
 *
 * 参考: 逐飞 CYT4BB 开源库 ekf.h
 *
 * 状态向量: X = [q0, q1, q2, q3]ᵀ  (四元数, 4×1)
 * 观测向量: Z = [ax, ay, az]ᵀ     (归一化加速度, 3×1)
 *
 * EKF 流程 (每 dt 秒执行一次):
 *   1. 读取 IMU 数据 (陀螺仪 + 加速度计)
 *   2. 构建状态转移矩阵 F (由角速度决定)
 *   3. 预测: X_ = F × X, P_ = F × P × Fᵀ + Q
 *   4. 计算观测矩阵 H (四元数对加速度的雅可比)
 *   5. 计算新息协方差 D = H × P_ × Hᵀ + R
 *   6. 马氏距离检验 (异常观测值剔除)
 *   7. 卡尔曼增益: K = P_ × Hᵀ × D⁻¹
 *   8. 状态更新: X = X_ + K × (Z - H × X_)
 *   9. 协方差更新: P = (I - K × H) × P_
 *  10. 四元数转欧拉角
 */

#ifndef __EKF_H
#define __EKF_H

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

/* ===================== 常量定义 ===================== */

/*
 * RAD_TO_DEG - 弧度转角度系数
 * 用途: 将 EKF 输出的四元数(弧度)转为欧拉角(度)
 * 计算: 180 / π ≈ 57.296
 */
#define RAD_TO_DEG  (57.295779513082320876798154814105f)

/*
 * PI - 圆周率
 * 用途: 陀螺仪 LSB 转 rad/s 时使用
 * 注: STM32 HAL 环境下可能已定义，用 #ifndef 防止重复
 */
#ifndef PI
#define PI          (3.1415926535897932384626433832795f)
#endif

/*
 * EKF_DT - EKF 预测步长 (秒)
 * 用途: 构建状态转移矩阵 F 时使用，决定陀螺仪积分的离散时间间隔
 * 调整建议: 需与主循环实际周期一致
 *   - 0.01s (100Hz) : 当前值，与主循环 20ms tick 配合 (实际 20ms 会自动 clamp)
 *   - 0.001s (1kHz) : 逐飞原始值，配合 1ms 定时器中断
 */
#define EKF_DT      (0.01f)

/*
 * ACC_LPF_K - 加速度计一阶低通滤波系数
 * 用途: 对加速度计原始数据做平滑，抑制高频振动/冲击
 * 计算: acc_filtered = K × acc_new + (1-K) × acc_last
 * 调整建议:
 *   - K=1.0  : 不滤波，直接使用原始数据 (响应最快，噪声最大)
 *   - K=0.7  : 当前值，轻度滤波 (逐飞默认值)
 *   - K=0.1  : 强滤波，平滑但延迟大 (适合低频机械结构)
 * 注意: K 值过小会导致观测延迟，EKF 跟不上快速运动
 */
#define ACC_LPF_K   (0.7f)

/*
 * MAHALANOBIS_THRESHOLD - 马氏距离阈值
 * 用途: 判断加速度计观测值是否异常 (如碰撞、急加减速时加速度计偏离重力方向)
 * 计算: r = (Z - h(X_))ᵀ × D⁻¹ × (Z - h(X_))
 *   - r < 阈值 : 观测正常，执行观测更新 (用加速度计修正姿态)
 *   - r > 阈值 : 观测异常，跳过更新 (仅用陀螺仪预测，防止错误修正)
 * 调整建议:
 *   - 0.1~0.3 : 非常严格，几乎只信任陀螺仪 (适合高动态/振动场景)
 *   - 0.4     : 当前值，较严格，允许轻微加速度计修正
 *   - 1.0~2.0 : 较宽松，更多依赖加速度计 (适合静态/慢速场景)
 *   - 4.0     : 逐飞原始值，基本不过滤 (始终接受加速度计更新)
 */
#define MAHALANOBIS_THRESHOLD (0.4f)

/*
 * Q_DIAG - 过程噪声协方差 Q 的对角元素值
 * 用途: 预测步骤中协方差更新 P_ = F×P×Fᵀ + Q，Q 控制预测不确定性增长速度
 * 物理意义: 值越大 → 陀螺仪模型越不可信 → 协方差增长越快 → 观测更新权重越大
 * 调整建议:
 *   - 0.001  : 陀螺仪非常可信，协方差增长慢 (适合高精度陀螺仪)
 *   - 0.005  : 逐飞原始值
 *   - 0.1    : 当前值，对陀螺仪有一定不信任 (适合 MPU6050 等消费级传感器)
 *   - 1.0    : 陀螺仪完全不可信，几乎完全依赖加速度计
 * 影响: Q 增大 → pitch/roll 收敛更快，但对陀螺仪漂移更敏感
 */
#define Q_DIAG      (0.1f)

/*
 * R_DIAG - 观测噪声协方差 R 的对角元素值
 * 用途: 新息协方差 D = H×P_×Hᵀ + R，R 控制加速度计测量噪声大小
 * 物理意义: 值越大 → 加速度计测量越不可信 → 卡尔曼增益越小 → 观测更新越弱
 * 调整建议:
 *   - 0.01   : 加速度计非常可信，观测更新权重很大
 *   - 0.2    : 当前值，轻度不信任加速度计
 *   - 1.0    : 逐飞原始值，对加速度计较不信任
 *   - 100.0  : 加速度计几乎不可信，几乎不用加速度计修正
 * 影响: R 增大 → 姿态跟踪更平滑/抗振动，但静止时收敛变慢
 *
 * 与 Q 的关系: Q/R 比值决定了"信陀螺仪"还是"信加速度计"
 *   - Q 小 R 大 → 信陀螺仪 (动态响应好，静止可能漂移)
 *   - Q 大 R 小 → 信加速度计 (静止准确，动态响应差)
 */
#define R_DIAG      (0.5f)

/*
 * P_DIAG - 初始误差协方差 P 的对角元素值
 * 用途: EKF 启动时的初始不确定度，影响首次观测更新的收敛速度
 * 物理意义: 值越大 → 初始状态越不确定 → 第一次加速度计观测的权重越大
 * 调整建议:
 *   - 10.0   : 较小初始不确定，收敛稍慢但更平滑
 *   - 100.0  : 当前值，较快收敛
 *   - 1000000: 逐飞原始值，几乎瞬间收敛 (启动时完全信任加速度计)
 * 注意: 该值只在 EKF_Init() 后的第一轮更新起作用，
 *       几次迭代后 P 会收敛到稳定值，P_DIAG 不再有影响
 */
#define P_DIAG      (100.0f)

/* ===================== 矩阵运算 (简化版, 仅 4×4) ===================== */

/*
 * 4×4 矩阵结构体
 * data[i][j]: 第 i 行, 第 j 列
 */
typedef struct {
    int rows;
    int cols;
    float data[4][4];
} Matrix4;

/* 4×1 向量结构体 */
typedef struct {
    float data[4];
} Vector4;

/* 3×1 向量结构体 */
typedef struct {
    float data[3];
} Vector3;

/* 3×3 矩阵结构体 */
typedef struct {
    float data[3][3];
} Matrix3;

/* ===================== 矩阵运算函数 ===================== */

static inline void Matrix_Init4(Matrix4 *m, int rows, int cols)
{
    m->rows = rows;
    m->cols = cols;
    memset(m->data, 0, sizeof(m->data));
}

static inline void Matrix_From_Array4(Matrix4 *m, const float *array, int rows, int cols)
{
    Matrix_Init4(m, rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            m->data[i][j] = array[i * cols + j];
        }
    }
}

static inline void Matrix_From_Array3(Matrix3 *m, const float *array, int rows, int cols)
{
    memset(m->data, 0, sizeof(m->data));
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            m->data[i][j] = array[i * cols + j];
        }
    }
}

/* 4×4 矩阵乘法: C = A × B */
static inline Matrix4 Matrix_Multiply4(Matrix4 *A, Matrix4 *B)
{
    Matrix4 C;
    Matrix_Init4(&C, A->rows, B->cols);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < B->cols; j++) {
            for (int k = 0; k < A->cols; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }
    return C;
}

/* 4×4 + 4×4 矩阵加法 */
static inline Matrix4 Matrix_Add4(Matrix4 *A, Matrix4 *B)
{
    Matrix4 C;
    Matrix_Init4(&C, A->rows, A->cols);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            C.data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }
    return C;
}

/* 4×4 - 4×4 矩阵减法 */
static inline Matrix4 Matrix_Subtract4(Matrix4 *A, Matrix4 *B)
{
    Matrix4 C;
    Matrix_Init4(&C, A->rows, A->cols);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            C.data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }
    return C;
}

/* 4×4 矩阵转置 */
static inline Matrix4 Matrix_Transpose4(Matrix4 *A)
{
    Matrix4 AT;
    Matrix_Init4(&AT, A->cols, A->rows);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            AT.data[j][i] = A->data[i][j];
        }
    }
    return AT;
}

/* 4×4 单位矩阵 */
static inline void Matrix_Identity4(Matrix4 *I, int size)
{
    Matrix_Init4(I, size, size);
    for (int i = 0; i < size; i++) {
        I->data[i][i] = 1.0f;
    }
}

/* 4×1 向量归一化 (Fast InvSqrt) */
static inline float Fast_InvSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;
    i = 0x5f375a86 - (i >> 1);
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);
    return x;
}

static inline void Vector4_Normalize(Vector4 *v)
{
    float norm_sq = 0.0f;
    for (int i = 0; i < 4; i++) {
        norm_sq += v->data[i] * v->data[i];
    }
    float inv_norm = Fast_InvSqrt(norm_sq);
    for (int i = 0; i < 4; i++) {
        v->data[i] *= inv_norm;
    }
}

/* 3×1 向量归一化 */
static inline void Vector3_Normalize(Vector3 *v)
{
    float norm_sq = 0.0f;
    for (int i = 0; i < 3; i++) {
        norm_sq += v->data[i] * v->data[i];
    }
    float inv_norm = Fast_InvSqrt(norm_sq);
    for (int i = 0; i < 3; i++) {
        v->data[i] *= inv_norm;
    }
}

/* 3×3 矩阵求逆 (高斯-约当消元, 仅适用于对称正定矩阵) */
static inline int Matrix3_Inverse(float A[3][3], float invA[3][3])
{
    const float THRESHOLD = 1e-6f;
    float aug[3][6];  // [A | I]

    // 构造增广矩阵 [A | I]
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            aug[i][j]     = A[i][j];
            aug[i][j + 3] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 高斯-约当消元
    for (int i = 0; i < 3; i++) {
        // 选主元
        int max_row = i;
        for (int j = i + 1; j < 3; j++) {
            if (fabs(aug[j][i]) > fabs(aug[max_row][i])) {
                max_row = j;
            }
        }

        // 奇异检测
        if (fabs(aug[max_row][i]) < THRESHOLD) {
            return 1;  // 奇异
        }

        // 行交换
        if (max_row != i) {
            for (int j = 0; j < 6; j++) {
                float tmp = aug[i][j];
                aug[i][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }

        // 归一化
        float pivot = aug[i][i];
        for (int j = 0; j < 6; j++) {
            aug[i][j] /= pivot;
        }

        // 消去
        for (int j = 0; j < 3; j++) {
            if (j != i) {
                float factor = aug[j][i];
                for (int k = 0; k < 6; k++) {
                    aug[j][k] -= factor * aug[i][k];
                }
            }
        }
    }

    // 提取逆矩阵
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            invA[i][j] = aug[i][j + 3];
        }
    }
    return 0;
}

/* ===================== IMU 数据结构 ===================== */

/*
 * imu_t - IMU 滤波后数据
 *
 * gyro: 陀螺仪角速度 (rad/s)
 * acc : 加速度计 (原始 ADC 值, 低通滤波后)
 */
typedef struct {
    float gyro_x;   // X轴角速度 (rad/s, 横滚轴)
    float gyro_y;   // Y轴角速度 (rad/s, 俯仰轴)
    float gyro_z;   // Z轴角速度 (rad/s, 偏航轴)
    float acc_x;    // X轴加速度 (滤波后)
    float acc_y;    // Y轴加速度
    float acc_z;    // Z轴加速度
} imu_t;

/*
 * EulerAngles - 欧拉角 (单位: 度)
 *
 * pitch: 俯仰角 (绕Y轴), 正值=前倾
 * roll : 横滚角 (绕X轴), 正值=右倾
 * yaw  : 偏航角 (绕Z轴), 正值=右转
 */
typedef struct {
    float pitch;
    float roll;
    float yaw;
} EulerAngles;

/*
 * Quaternion - 四元数
 *
 * q0: 标量分量
 * q1,q2,q3: 向量分量
 *
 * 四元数姿态表示, 避免万向节锁
 */
typedef struct {
    float q0;   // 标量
    float q1;   // x
    float q2;   // y
    float q3;   // z
} Quaternion;

/*
 * GyroOffset - 陀螺仪零偏
 */
typedef struct {
    float x;
    float y;
    float z;
} GyroOffset;

/* ===================== 全局变量 ===================== */

/* EKF 状态向量: 四元数 [q0, q1, q2, q3]ᵀ */
extern Vector4 ekf_state;

/* EKF 输出的欧拉角 (pitch/roll/yaw, 单位: 度) */
extern EulerAngles euler_angle;

/* IMU 滤波后数据 */
extern imu_t imu_data;

/* 马氏距离检验结果 */
extern float ekf_mahalDist;

/* ===================== INS 共享变量 ===================== */

/* INS 四元数在 ins.c 中定义为 static，此处不再 extern 声明 */

/* INS 陀螺仪零偏 (由 ekf.c 定义，ins.c 使用) */
extern GyroOffset gyro_offset;

/* INS 欧拉角 (由 ins.c 定义，ekf.c 不使用) */
extern EulerAngles ins_euler;

/* INS 状态 (由 ins.c 定义) */
typedef struct {
    Quaternion quat;
    EulerAngles euler;
    float vx, vy, vz;
    float px, py, pz;
    float gyro_yaw_integrated;
    float accel_yaw;
    struct {
        float acc_x, acc_y, acc_z;
        float gyro_x, gyro_y, gyro_z;
    } raw;
} INSState;

extern INSState ins_state;

/* INS Roll 平滑滤波 (由 ins.c 定义) */
extern float ins_filtered_roll;

/* ===================== 函数声明 ===================== */

/*
 * EKF_Init - EKF 初始化
 *
 * 调用一次即可, 在 main() 中初始化
 */
void EKF_Init(void);

/*
 * EKF_UpData - EKF 一步更新
 *
 * 每 EKF_DT 秒 (由定时器中断) 调用一次
 * 读取 IMU → 预测 → 更新 → 输出欧拉角
 */
void EKF_UpData(void);

/*
 * EKF_GetAngles - 获取滤波后的欧拉角
 *
 * 返回: EulerAngles 结构体指针
 */
const EulerAngles* EKF_GetAngles(void);

/*
 * EKF_CalibrateGyro - 陀螺仪零漂校准
 *
 * 将MPU6050水平静止放置后调用，函数会自动采集样本计算零漂值
 * @param samples  采样次数 (建议100-500)
 * @param delay_ms 每次采样间隔 (ms)
 */
void EKF_CalibrateGyro(uint16_t samples, uint16_t delay_ms);

/*
 * IMU_UpdateBuffer - 更新 IMU 数据缓冲区 (主循环调用)
 *
 * 由主循环调用，将读取到的原始 IMU 数据写入缓冲区
 * @param gyro_raw[3] 陀螺仪原始值 (LSB)
 * @param acc_raw[3]  加速度计原始值 (g)
 */
void IMU_UpdateBuffer(float gyro_raw[3], float acc_raw[3]);

/*
 * EKF_Update_NoRead - EKF 更新 (不读取 IMU, 主循环调用)
 *
 * 由主循环调用，使用预更新的 imu_data 运行 EKF 算法
 * @return 0: 正常更新, 1: 矩阵奇异跳过, 2: 马氏距离过大跳过
 */
uint8_t EKF_Update_NoRead(void);

/*
 * IMU_Read_Raw - 读取 IMU 原始数据 (需根据实际硬件实现)
 *
 * 此函数需要用户根据具体 IMU 驱动实现:
 *   - 读取陀螺仪原始值 (°/s)
 *   - 读取加速度计原始值 (ADC)
 *
 * 示例 (MPU6050):
 *   void IMU_Read_Raw(float *gyro, float *acc) {
 *       MPU6050_Read_Gyro(&gyro[0], &gyro[1], &gyro[2]);
 *       MPU6050_Read_Acc(&acc[0], &acc[1], &acc[2]);
 *   }
 */
void IMU_Read_Raw(float gyro[3], float acc[3]);

#endif /* __EKF_H */
