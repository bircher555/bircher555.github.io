/*
 * kalman.h - 一阶卡尔曼滤波器
 *
 * 适用场景：平滑单一传感器信号（编码器速度、ADC采样值等）
 *
 * 模型:
 *   状态向量: X = [x, vx]ᵀ  (位置 + 速度)
 *   观测向量: Z = x_measured
 *
 * 使用方法:
 *   KalmanFilter kf;
 *   Kalman_Init(&kf, dt);
 *   while (1) {
 *       Kalman_Predict(&kf);
 *       float filtered = Kalman_Update(&kf, measurement);
 *   }
 */

#ifndef __KALMAN_H
#define __KALMAN_H

#include "main.h"

/*
 * 一阶卡尔曼滤波器结构体
 *
 * 状态: x = 估计值, vx = 估计速度
 * P   = 估计误差协方差 (对角矩阵，仅存对角元素)
 * Q   = 过程噪声协方差 (信任模型)
 * R   = 观测噪声协方差 (信任测量)
 */
typedef struct {
    float x;      // 估计状态 (位置)
    float vx;     // 估计速度

    float P[2];   // 误差协方差对角元素: [P00, P11]

    float Q0;     // 过程噪声: 位置
    float Q1;     // 过程噪声: 速度
    float R;      // 观测噪声

    float dt;     // 时间步长 (秒)
} KalmanFilter;

void Kalman_Init(KalmanFilter *kf, float dt);
void Kalman_InitParam(KalmanFilter *kf, float dt, float Q0, float Q1, float R);
void Kalman_Predict(KalmanFilter *kf);
float Kalman_Update(KalmanFilter *kf, float z);

#endif
