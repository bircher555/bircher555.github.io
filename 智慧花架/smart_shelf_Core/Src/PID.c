/*
 * PID.c - PID控制器模块实现
 *
 * ===================== 位置式PID vs 增量式PID =====================
 *
 * | 特性         | 增量式PID              | 位置式PID             |
 * |--------------|-----------------------|-----------------------|
 * | 适用场景     | 速度控制               | 位置控制               |
 * | 输出形式     | 增量累加               | 直接输出               |
 * | 抗干扰能力   | 强                     | 弱                    |
 * | 启动冲击     | 小                     | 可能较大              |
 * | 积分饱和     | 不易饱和               | 需要积分限幅           |
 * | 计算误差影响 | 单次误差影响小          | 单次误差持续影响       |
 * | 推荐用于     | 驱动电机（速度闭环）    | 舵机/步进电机（位置）   |
 *
 * ===================== 函数参数说明 =====================
 *
 * Motor_PID_Init() 参数：
 *   - type: PID类型
 *       PID_TYPE_INCREMENTAL (1) - 增量式，推荐用于驱动电机速度控制
 *       PID_TYPE_POSITION (0)    - 位置式，推荐用于舵机位置控制
 *
 *   - kp: 比例系数
 *       增量式推荐：1.0 ~ 2.0
 *       位置式推荐：2.0 ~ 3.0
 *       作用：决定响应速度，过大会震荡
 *
 *   - ki: 积分系数
 *       增量式推荐：0.2 ~ 0.5
 *       位置式推荐：0.3 ~ 0.8
 *       作用：消除稳态误差，过大会超调
 *
 *   - kd: 微分系数
 *       增量式推荐：0.01 ~ 0.1
 *       位置式推荐：0.05 ~ 0.2
 *       作用：抑制震荡，过大会对干扰敏感
 *
 *   - maxIntegral: 积分限幅（仅位置式有效）
 *       推荐值：500 ~ 2000
 *       作用：防止积分饱和，增量式填0即可
 *
 *   - maxOutput: 输出限幅
 *       增量式：增量限幅，推荐 300 ~ 800
 *       位置式：总输出限幅，推荐 1000 ~ 7200（对应PWM最大值）
 *
 *   - deadzone: 死区范围
 *       推荐值：1.0 ~ 10.0
 *       作用：误差小于此值时不控制，防止抖动
 */

#include "PID.h"
#include <math.h>

/**
 * @brief  初始化PID控制器
 */
void PID_Init(PID *pid, float kp, float ki, float kd,
              float maxIntegral, float maxOutput, float deadzone)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxIntegral = maxIntegral;
    pid->maxOutput = maxOutput;
    pid->deadzone = deadzone;

    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  PID计算（位置式）
 */
float PID_Calculate(PID *pid, float target, float feedback)
{
    /* 计算误差 */
    pid->error = target - feedback;

    /* 死区处理：误差过小时视为0，防止抖动 */
    if (fabsf(pid->error) < pid->deadzone)
    {
        pid->error = 0.0f;
    }

    /* 积分项累加 */
    pid->integral += pid->error;

    /* 积分限幅，防止积分饱和 */
    LIMIT(pid->integral, -pid->maxIntegral, pid->maxIntegral);

    /* 计算PID输出：P + I + D */
    pid->output = pid->kp * pid->error
                + pid->ki * pid->integral
                + pid->kd * (pid->error - pid->lastError);

    /* 输出限幅 */
    LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);

    /* 保存当前误差供下次微分使用 */
    pid->lastError = pid->error;

    return pid->output;
}

/**
 * @brief  清除PID积分项和误差
 */
void PID_Clear(PID *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/* ===================== 增量式PID实现 ===================== */

/**
 * @brief  初始化增量式PID控制器
 */
void PID_Incremental_Init(PID_Incremental *pid, float kp, float ki, float kd,
                          float maxOutput, float deadzone)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->maxOutput = maxOutput;
    pid->deadzone = deadzone;

    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  增量式PID计算
 * @note   增量式PID公式：
 *         ΔU(k) = Kp[e(k) - e(k-1)] + Ki*e(k) + Kd[e(k) - 2e(k-1) + e(k-2)]
 *         实际控制量：U(k) = U(k-1) + ΔU(k)
 */
float PID_Incremental_Calculate(PID_Incremental *pid, float target, float feedback)
{
    /* 计算当前误差 */
    pid->error = target - feedback;

    /* 死区处理：误差过小时视为0，防止抖动 */
    if (fabsf(pid->error) < pid->deadzone)
    {
        pid->error = 0.0f;
    }

    /* 增量式PID计算：
     * P项：Kp * [e(k) - e(k-1)]
     * I项：Ki * e(k)
     * D项：Kd * [e(k) - 2*e(k-1) + e(k-2)]
     */
    pid->output = pid->kp * (pid->error - pid->lastError)
                + pid->ki * pid->error
                + pid->kd * (pid->error - 2.0f * pid->lastError + pid->prevError);

    /* 输出增量限幅 */
    LIMIT(pid->output, -pid->maxOutput, pid->maxOutput);

    /* 更新误差历史 */
    pid->prevError = pid->lastError;
    pid->lastError = pid->error;

    return pid->output;
}

/**
 * @brief  清除增量式PID误差
 */
void PID_Incremental_Clear(PID_Incremental *pid)
{
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->prevError = 0.0f;
    pid->output = 0.0f;
}

/* ===================== 电机PID管理实现 ===================== */

/**
 * @brief  初始化电机PID控制器
 */
void Motor_PID_Init(Motor_PID_Controller *motor_pid, PID_Type type,
                    float kp, float ki, float kd,
                    float maxIntegral, float maxOutput, float deadzone)
{
    motor_pid->type = type;
    motor_pid->accumulated_output = 0.0f;

    if (type == PID_TYPE_POSITION)
    {
        PID_Init(&motor_pid->position, kp, ki, kd, maxIntegral, maxOutput, deadzone);
    }
    else  // PID_TYPE_INCREMENTAL
    {
        PID_Incremental_Init(&motor_pid->incremental, kp, ki, kd, maxOutput, deadzone);
    }
}

/**
 * @brief  设置电机PID参数
 */
void Motor_PID_SetParams(Motor_PID_Controller *motor_pid, float kp, float ki, float kd)
{
    if (motor_pid->type == PID_TYPE_POSITION)
    {
        if (kp >= 0.0f) motor_pid->position.kp = kp;
        if (ki >= 0.0f) motor_pid->position.ki = ki;
        if (kd >= 0.0f) motor_pid->position.kd = kd;
    }
    else  // PID_TYPE_INCREMENTAL
    {
        if (kp >= 0.0f) motor_pid->incremental.kp = kp;
        if (ki >= 0.0f) motor_pid->incremental.ki = ki;
        if (kd >= 0.0f) motor_pid->incremental.kd = kd;
    }
}

/**
 * @brief  设置电机PID输出限幅
 */
void Motor_PID_SetMaxOutput(Motor_PID_Controller *motor_pid, float maxOutput)
{
    if (motor_pid->type == PID_TYPE_POSITION)
    {
        motor_pid->position.maxOutput = maxOutput;
    }
    else  // PID_TYPE_INCREMENTAL
    {
        motor_pid->incremental.maxOutput = maxOutput;
    }
}

/**
 * @brief  设置电机PID积分限幅（仅位置式有效）
 */
void Motor_PID_SetMaxIntegral(Motor_PID_Controller *motor_pid, float maxIntegral)
{
    if (motor_pid->type == PID_TYPE_POSITION)
    {
        motor_pid->position.maxIntegral = maxIntegral;
    }
}

/**
 * @brief  电机PID计算（自动根据类型选择算法）
 */
float Motor_PID_Calculate(Motor_PID_Controller *motor_pid, float target, float feedback)
{
    float output;

    if (motor_pid->type == PID_TYPE_POSITION)
    {
        /* 位置式PID：直接返回输出值 */
        output = PID_Calculate(&motor_pid->position, target, feedback);
    }
    else  // PID_TYPE_INCREMENTAL
    {
        /* 增量式PID：累加增量到输出 */
        float delta = PID_Incremental_Calculate(&motor_pid->incremental, target, feedback);
        motor_pid->accumulated_output += delta;

        /* 累积输出限幅 */
        LIMIT(motor_pid->accumulated_output, -100.0f, 100.0f);
        output = motor_pid->accumulated_output;
    }

    return output;
}

/**
 * @brief  清除电机PID状态
 */
void Motor_PID_Clear(Motor_PID_Controller *motor_pid)
{
    motor_pid->accumulated_output = 0.0f;

    if (motor_pid->type == PID_TYPE_POSITION)
    {
        PID_Clear(&motor_pid->position);
    }
    else  // PID_TYPE_INCREMENTAL
    {
        PID_Incremental_Clear(&motor_pid->incremental);
    }
}
