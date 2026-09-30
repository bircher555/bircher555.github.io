/*
 * ekf_pos_fusion.h - 里程计 + IMU EKF 位置融合
 *
 * 将轮式里程计（编码器）与 IMU 惯性测量融合，
 * 输出高精度的位置、速度、航向估计。
 *
 * 状态向量: X = [x, y, heading, vx, vy, omega]ᵀ  (6×1)
 *   x, y       : 世界坐标系位置 (m)
 *   heading    : 航向角 (rad)
 *   vx, vy    : 车身坐标系线速度 (m/s)
 *   omega      : 角速度 (rad/s)
 *
 * EKF 预测: 编码器轮速积分
 * EKF 更新: IMU 加速度（检测打滑）+ 陀螺仪 yaw（修正航向漂移）
 */

#ifndef __EKF_POS_FUSION_H
#define __EKF_POS_FUSION_H

/* 复用 ekf.h 中的类型 (Vector3), 函数 (Fast_InvSqrt, Matrix3_Inverse) */
#include "ekf.h"
#include <math.h>
#include <string.h>

/* ===================== 矩阵运算 (6×6) ===================== */

/*
 * 6×6 矩阵结构体
 * data[i][j]: 第 i 行, 第 j 列
 */
typedef struct {
    int rows;
    int cols;
    float data[6][6];
} Matrix6;

/* 6×1 向量结构体 */
typedef struct {
    float data[6];
} Vector6;

/* 2×1 向量结构体 */
typedef struct {
    float data[2];
} Vector2;

/* ===================== 矩阵运算函数 ===================== */

/* 注意: Fast_InvSqrt, Vector3, Matrix3_Inverse 已由 ekf.h 提供 */

/* 6×6 矩阵初始化 */
static inline void Matrix6_Init(Matrix6 *m, int rows, int cols)
{
    m->rows = rows;
    m->cols = cols;
    memset(m->data, 0, sizeof(m->data));
}

/* 6×6 矩阵乘法: C = A × B */
static inline Matrix6 Matrix6_Multiply(Matrix6 *A, Matrix6 *B)
{
    Matrix6 C;
    Matrix6_Init(&C, A->rows, B->cols);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < B->cols; j++) {
            for (int k = 0; k < A->cols; k++) {
                C.data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }
    return C;
}

/* 6×6 + 6×6 矩阵加法 */
static inline Matrix6 Matrix6_Add(Matrix6 *A, Matrix6 *B)
{
    Matrix6 C;
    Matrix6_Init(&C, A->rows, A->cols);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }
    return C;
}

/* 6×6 - 6×6 矩阵减法 */
static inline Matrix6 Matrix6_Subtract(Matrix6 *A, Matrix6 *B)
{
    Matrix6 C;
    Matrix6_Init(&C, A->rows, A->cols);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            C.data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }
    return C;
}

/* 6×6 矩阵转置 */
static inline Matrix6 Matrix6_Transpose(Matrix6 *A)
{
    Matrix6 AT;
    Matrix6_Init(&AT, A->cols, A->rows);
    for (int i = 0; i < A->rows; i++) {
        for (int j = 0; j < A->cols; j++) {
            AT.data[j][i] = A->data[i][j];
        }
    }
    return AT;
}

/* 6×6 单位矩阵 */
static inline void Matrix6_Identity(Matrix6 *I, int size)
{
    Matrix6_Init(I, size, size);
    for (int i = 0; i < size; i++) {
        I->data[i][i] = 1.0f;
    }
}

/* 6×6 对角矩阵 (6个对角元素) */
static inline void Matrix6_Diagonal(Matrix6 *D, const float diag[6])
{
    Matrix6_Init(D, 6, 6);
    for (int i = 0; i < 6; i++) {
        D->data[i][i] = diag[i];
    }
}

/* 6×1 向量归一化 */
static inline void Vector6_Normalize(Vector6 *v)
{
    float norm_sq = 0.0f;
    for (int i = 0; i < 6; i++) {
        norm_sq += v->data[i] * v->data[i];
    }
    float inv_norm = Fast_InvSqrt(norm_sq);
    for (int i = 0; i < 6; i++) {
        v->data[i] *= inv_norm;
    }
}

/* 2×2 矩阵求逆 (对称正定矩阵) */
static inline int Matrix2_Inverse(float A[2][2], float invA[2][2])
{
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabsf(det) < 1e-6f) return 1;
    float inv_det = 1.0f / det;
    invA[0][0] =  A[1][1] * inv_det;
    invA[0][1] = -A[0][1] * inv_det;
    invA[1][0] = -A[1][0] * inv_det;
    invA[1][1] =  A[0][0] * inv_det;
    return 0;
}

/* ===================== EKF 噪声矩阵默认值 ===================== */

/* 过程噪声 Q 对角元素 (对编码器速度积分的信任度)
 * 值越大 → 对预测模型信任度越低，更依赖观测修正 */
#define PF_Q_X        0.01f
#define PF_Q_Y        0.01f
#define PF_Q_HEADING  0.001f
#define PF_Q_VX       0.1f
#define PF_Q_VY       0.1f
#define PF_Q_OMEGA    0.01f

/* 观测噪声 R (对各观测源的信任度，越大越不信任) */

/* 编码器速度观测 */
#define PF_R_ENC_VX     0.5f
#define PF_R_ENC_VY     0.5f
#define PF_R_ENC_OMEGA  0.1f

/* IMU 加速度观测 (世界坐标系) */
#define PF_R_IMU_AX    1.0f
#define PF_R_IMU_AY    1.0f

/* 陀螺仪 yaw 观测 */
#define PF_R_YAW       0.1f

/* 马氏距离阈值 (检测 IMU 异常) */
#define PF_MAHAL_THRESHOLD  1.0f

/* ===================== 融合状态输出 ===================== */

typedef struct {
    float x;         // 世界坐标系 X (m)
    float y;         // 世界坐标系 Y (m)
    float heading;    // 航向角 (rad)
    float vx;        // 车身坐标系 X 速度 (m/s)
    float vy;        // 车身坐标系 Y 速度 (m/s)
    float omega;     // 角速度 (rad/s)
    float v_linear;  // 合线速度 (m/s)
} PosFusionState;

/* ===================== 全局变量 (外部引用) ===================== */

/* 编码器里程计输出的速度 (由 odometry.c 提供) */
extern float odom_fusion_vx_enc;
extern float odom_fusion_vy_enc;
extern float odom_fusion_omega_enc;

/* ===================== 函数声明 ===================== */

/*
 * PosFusion_Init - 融合器初始化
 *
 * @param x0, y0, heading0: 初始位置和航向
 */
void PosFusion_Init(float x0, float y0, float heading0);

/*
 * PosFusion_Update - EKF 一步融合更新
 *
 * 在 TIM6 10ms 中断中调用
 *
 * @param vx_enc, vy_enc, omega_enc : 编码器测得的车身速度 (m/s, rad/s)
 * @param acc_x, acc_y              : IMU 加速度 (车身坐标系, m/s²)
 * @param yaw_gyro                  : 陀螺仪 yaw (rad)
 * @param dt                        : 时间步长 (秒)
 */
void PosFusion_Update(float vx_enc, float vy_enc, float omega_enc,
                     float acc_x, float acc_y, float yaw_gyro, float dt);

/*
 * PosFusion_GetState - 获取融合后的状态
 *
 * @return PosFusionState 指针
 */
const PosFusionState* PosFusion_GetState(void);

#endif /* __EKF_POS_FUSION_H */
