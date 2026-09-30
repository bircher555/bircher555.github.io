/*
 * PID.h - PID控制器模块
 *
 * 提供单环PID和级联PID控制功能，用于电机速度/位置控制
 */

#ifndef __PID_H
#define __PID_H

#include "main.h"

/* 通用限幅宏：将 x 限制在 [min, max] 范围内 */
#define LIMIT(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

/* ===================== 单环PID结构体（位置式）===================== */
typedef struct
{
    float kp, ki, kd;        // 比例、积分、微分系数
    float error, lastError;  // 当前误差，上一次误差（用于微分项计算）
    float integral;          // 积分累计值
    float maxIntegral;       // 积分限幅（防止积分饱和）
    float output;            // PID输出值
    float maxOutput;         // 输出限幅
    float deadzone;          // 死区范围：误差绝对值小于此值时视为0，防止抖动
} PID;

/* ===================== 增量式PID结构体 ===================== */
typedef struct
{
    float kp, ki, kd;              // 比例、积分、微分系数
    float error, lastError, prevError;  // 当前误差、上次误差、上上次误差
    float output;                  // PID输出增量
    float maxOutput;               // 输出增量限幅
    float deadzone;                // 死区范围
} PID_Incremental;

/* ===================== 函数声明 ===================== */

/**
 * @brief  初始化PID控制器
 * @param  pid: PID结构体指针
 * @param  kp, ki, kd: PID三个系数
 * @param  maxIntegral: 积分限幅值
 * @param  maxOutput: 输出限幅值
 * @param  deadzone: 死区范围
 */
void PID_Init(PID *pid, float kp, float ki, float kd,
              float maxIntegral, float maxOutput, float deadzone);

/**
 * @brief  PID计算（位置式）
 * @param  pid: PID结构体指针
 * @param  target: 目标值
 * @param  feedback: 反馈值
 * @return PID输出值
 */
float PID_Calculate(PID *pid, float target, float feedback);

/**
 * @brief  清除PID积分项和误差
 * @param  pid: PID结构体指针
 */
void PID_Clear(PID *pid);

/**
 * @brief  初始化增量式PID控制器
 * @param  pid: 增量式PID结构体指针
 * @param  kp, ki, kd: PID三个系数
 * @param  maxOutput: 输出增量限幅值
 * @param  deadzone: 死区范围
 */
void PID_Incremental_Init(PID_Incremental *pid, float kp, float ki, float kd,
                          float maxOutput, float deadzone);

/**
 * @brief  增量式PID计算
 * @param  pid: 增量式PID结构体指针
 * @param  target: 目标值
 * @param  feedback: 反馈值
 * @return PID输出增量（需要累加到实际控制量上）
 */
float PID_Incremental_Calculate(PID_Incremental *pid, float target, float feedback);

/**
 * @brief  清除增量式PID误差
 * @param  pid: 增量式PID结构体指针
 */
void PID_Incremental_Clear(PID_Incremental *pid);

/* ===================== 电机PID管理 ===================== */

/* PID类型枚举 */
typedef enum
{
    PID_TYPE_POSITION = 0,    // 位置式PID
    PID_TYPE_INCREMENTAL = 1  // 增量式PID
} PID_Type;

/* 电机PID控制器结构体 */
typedef struct
{
    PID_Type type;              // PID类型
    PID position;               // 位置式PID
    PID_Incremental incremental; // 增量式PID
    float accumulated_output;   // 累积输出（增量式用）
} Motor_PID_Controller;

/**
 * @brief  初始化电机PID控制器
 * @param  motor_pid: 电机PID控制器指针
 * @param  type: PID类型（PID_TYPE_POSITION 或 PID_TYPE_INCREMENTAL）
 * @param  kp, ki, kd: PID三个系数
 * @param  maxIntegral: 积分限幅值（仅位置式有效）
 * @param  maxOutput: 输出限幅值
 * @param  deadzone: 死区范围
 */
void Motor_PID_Init(Motor_PID_Controller *motor_pid, PID_Type type,
                    float kp, float ki, float kd,
                    float maxIntegral, float maxOutput, float deadzone);

/**
 * @brief  设置电机PID参数
 * @param  motor_pid: 电机PID控制器指针
 * @param  kp, ki, kd: PID三个系数（传入负值表示不修改该参数）
 */
void Motor_PID_SetParams(Motor_PID_Controller *motor_pid, float kp, float ki, float kd);

/**
 * @brief  设置电机PID输出限幅
 * @param  motor_pid: 电机PID控制器指针
 * @param  maxOutput: 输出限幅值
 */
void Motor_PID_SetMaxOutput(Motor_PID_Controller *motor_pid, float maxOutput);

/**
 * @brief  设置电机PID积分限幅（仅位置式有效）
 * @param  motor_pid: 电机PID控制器指针
 * @param  maxIntegral: 积分限幅值
 */
void Motor_PID_SetMaxIntegral(Motor_PID_Controller *motor_pid, float maxIntegral);

/**
 * @brief  电机PID计算（自动根据类型选择算法）
 * @param  motor_pid: 电机PID控制器指针
 * @param  target: 目标值
 * @param  feedback: 反馈值
 * @return 实际控制输出值（已处理增量累加）
 */
float Motor_PID_Calculate(Motor_PID_Controller *motor_pid, float target, float feedback);

/**
 * @brief  清除电机PID状态
 * @param  motor_pid: 电机PID控制器指针
 */
void Motor_PID_Clear(Motor_PID_Controller *motor_pid);

#endif /* __PID_H */
