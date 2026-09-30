/*
 * kalman.c - 一阶卡尔曼滤波器实现
 *
 * 模型: X_k = F * X_{k-1} + W
 *       Z_k = H * X_k + V
 *
 * 其中:
 *   F = [1 dt]  (状态转移矩阵)
 *       [0  1]
 *   H = [1 0]   (观测矩阵)
 *   W ~ N(0, Q) 过程噪声
 *   V ~ N(0, R) 观测噪声
 */

#include "kalman.h"
#include <math.h>

/*
 * Kalman_Init - 默认参数初始化
 *
 * dt: 时间步长 (秒)，例如 dt=0.01 表示 100Hz
 */
void Kalman_Init(KalmanFilter *kf, float dt)
{
    Kalman_InitParam(kf, dt, 0.1f, 0.5f, 10.0f);
}

/*
 * Kalman_InitParam - 自定义参数初始化
 *
 * 参数:
 *   Q0: 位置过程噪声 (越小越信任模型/预测)
 *   Q1: 速度过程噪声
 *   R : 观测噪声 (越小越信任观测)
 *
 * 调参建议:
 *   平滑为主: Q0=0.01, Q1=0.1,  R=50
 *   响应为主: Q0=1.0,  Q1=5.0,  R=5
 *   平衡   : Q0=0.1,  Q1=0.5,  R=10
 */
void Kalman_InitParam(KalmanFilter *kf, float dt, float Q0, float Q1, float R)
{
    kf->x = 0.0f;
    kf->vx = 0.0f;

    /* 初始误差协方差 (大方差表示初始不确定) */
    kf->P[0] = 1000.0f;
    kf->P[1] = 1000.0f;

    kf->Q0 = Q0;
    kf->Q1 = Q1;
    kf->R  = R;
    kf->dt = dt;
}

/*
 * Kalman_Predict - 预测步骤
 *
 * 根据运动模型预测下一状态:
 *   x  = x + vx * dt
 *   P  = F * P * F^T + Q
 *
 * 其中 F = [1 dt]
 *          [0  1]
 */
void Kalman_Predict(KalmanFilter *kf)
{
    /* 状态预测: 匀速模型 */
    kf->x  += kf->vx * kf->dt;

    /* 协方差预测: P = F * P * F^T + Q
     * P00_new = P00 + 2*dt*P01 + dt^2*P11 + Q0
     * P01_new = P01 + dt*P11
     * P10_new = P01 + dt*P11
     * P11_new = P11 + Q1
     */
    float P00 = kf->P[0];
    float P01 = kf->P[1];

    kf->P[0] = P00 + 2.0f * kf->dt * P01 + kf->dt * kf->dt * kf->P[1] + kf->Q0;
    kf->P[1] = P01 + kf->dt * kf->P[1] + kf->Q1;
}

/*
 * Kalman_Update - 更新步骤
 *
 * 融合观测值 z:
 *   K  = P * H^T / (H * P * H^T + R)
 *   X  = X + K * (z - H * X)
 *   P  = (I - K * H) * P
 *
 * 其中 H = [1, 0]
 *
 * 返回值: 滤波后的状态估计 x
 */
float Kalman_Update(KalmanFilter *kf, float z)
{
    /* 计算卡尔曼增益 K
     * K = P * H^T / (H * P * H^T + R)
     *   = [P00; P10] / (P00 + R)
     */
    float HPHt_R = kf->P[0] + kf->R;

    float K0 = kf->P[0] / HPHt_R;  // K0 = P00 / (P00 + R)
    float K1 = kf->P[1] / HPHt_R;  // K1 = P10 / (P00 + R)

    /* 新息 (观测 - 预测) */
    float innovation = z - kf->x;

    /* 状态更新 */
    kf->x  += K0 * innovation;
    kf->vx += K1 * innovation;

    /* 协方差更新: P = (I - K * H) * P
     * P00_new = (1 - K0) * P00
     * P01_new = (1 - K0) * P01
     * P10_new = -K1 * P00 + P10
     * P11_new = -K1 * P10 + P11
     */
    kf->P[0] = (1.0f - K0) * kf->P[0];
    kf->P[1] = (1.0f - K0) * kf->P[1] - K1 * kf->P[0];

    return kf->x;
}
