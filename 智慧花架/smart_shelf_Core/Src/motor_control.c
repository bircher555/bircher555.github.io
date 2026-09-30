/*
 * motor_control.c - 电机PID闭环控制模块实现
 */

#include "motor_control.h"
#include "motor.h"
#include "encoder.h"
#include "tim.h"

/* 电机PID控制器数组 */
static Motor_PID_Controller motor_pid[MOTOR_CONTROL_NUM];

/* 目标值数组（增量式：速度-100~100，位置式：角度-180~180） */
static float target_value[MOTOR_CONTROL_NUM] = {0};

/* 当前值数组（增量式：速度-100~100，位置式：角度-180~180） */
static float current_value[MOTOR_CONTROL_NUM] = {0};

/**
 * @brief  初始化电机控制系统
 */
void Motor_Control_Init(void)
{
    /* 初始化四个电机的PID控制器（使用增量式PID）
     * 参数说明：
     *   Kp=0.5, Ki=0.1, Kd=0.01  - 初始PID参数（需要根据实际调试）
     *   maxIntegral=0            - 增量式不需要积分限幅
     *   maxOutput=20.0           - 输出增量限幅
     *   deadzone=1.0             - 死区范围（RPM）
     */
    for (uint8_t i = 0; i < MOTOR_CONTROL_NUM; i++)
    {
        Motor_PID_Init(&motor_pid[i], PID_TYPE_INCREMENTAL,
                       1.5f, 0.3f, 0.05f, 0, 500.0f, 5.0f);
        target_value[i] = 0.0f;
        current_value[i] = 0.0f;
    }

    /* 启动定时器6用于PID控制循环（10ms周期） */
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief  设置电机目标值和PID参数
 */
void Motor_Control_SetPID(uint8_t motor_id, int8_t type, float target,
                          float kp, float ki, float kd, float max_output)
{
    if (motor_id >= MOTOR_CONTROL_NUM)
        return;

    /* 切换PID类型（如果需要） */
    if (type >= 0 && type != motor_pid[motor_id].type)
    {
        /* 清除旧的PID状态 */
        Motor_PID_Clear(&motor_pid[motor_id]);

        /* 重新初始化为新类型 */
        float old_kp = (motor_pid[motor_id].type == PID_TYPE_POSITION)
                       ? motor_pid[motor_id].position.kp
                       : motor_pid[motor_id].incremental.kp;
        float old_ki = (motor_pid[motor_id].type == PID_TYPE_POSITION)
                       ? motor_pid[motor_id].position.ki
                       : motor_pid[motor_id].incremental.ki;
        float old_kd = (motor_pid[motor_id].type == PID_TYPE_POSITION)
                       ? motor_pid[motor_id].position.kd
                       : motor_pid[motor_id].incremental.kd;
        float old_max = (motor_pid[motor_id].type == PID_TYPE_POSITION)
                        ? motor_pid[motor_id].position.maxOutput
                        : motor_pid[motor_id].incremental.maxOutput;

        Motor_PID_Init(&motor_pid[motor_id], (PID_Type)type,
                       old_kp, old_ki, old_kd, 0, old_max, 5.0f);
    }

    /* 设置目标值（增量式：速度-100~100，位置式：角度-180~180） */
    if (target >= -200.0f)  // 使用-200作为"不修改"的标志
    {
        if (motor_pid[motor_id].type == PID_TYPE_INCREMENTAL)
        {
            /* 增量式：限制在-100~100 */
            LIMIT(target, -100.0f, 100.0f);
        }
        else
        {
            /* 位置式：限制在-180~180 */
            LIMIT(target, -180.0f, 180.0f);
        }
        target_value[motor_id] = target;
    }

    /* 设置PID参数 */
    Motor_PID_SetParams(&motor_pid[motor_id], kp, ki, kd);

    /* 设置输出限幅 */
    if (max_output >= 0.0f)
    {
        Motor_PID_SetMaxOutput(&motor_pid[motor_id], max_output);
    }
}

/**
 * @brief  获取电机当前值
 */
float Motor_Control_GetSpeed(uint8_t motor_id)
{
    if (motor_id >= MOTOR_CONTROL_NUM)
        return 0.0f;

    return current_value[motor_id];
}

/**
 * @brief  获取电机PID输出值
 */
float Motor_Control_GetOutput(uint8_t motor_id)
{
    if (motor_id >= MOTOR_CONTROL_NUM)
        return 0.0f;

    return motor_pid[motor_id].accumulated_output;
}

/**
 * @brief  停止所有电机
 */
void Motor_Control_StopAll(void)
{
    for (uint8_t i = 0; i < MOTOR_CONTROL_NUM; i++)
    {
        Motor_Control_Stop(i);
    }
}

/**
 * @brief  停止指定电机
 */
void Motor_Control_Stop(uint8_t motor_id)
{
    if (motor_id >= MOTOR_CONTROL_NUM)
        return;

    target_value[motor_id] = 0.0f;
    Motor_PID_Clear(&motor_pid[motor_id]);
    Motor_Set(motor_id, 0);
}

/**
 * @brief  PID控制循环（在定时器中断中调用）
 */
void Motor_Control_Loop(void)
{
    for (uint8_t i = 0; i < MOTOR_CONTROL_NUM; i++)
    {
        /* 1. 读取当前值 */
        if (motor_pid[i].type == PID_TYPE_INCREMENTAL)
        {
            /* 增量式：读取编码器速度，转换为-100~100 */
            float rpm = Encoder_GetRPM(i, CONTROL_PERIOD_MS);
            // 假设最大转速为300 RPM，映射到-100~100
            current_value[i] = (rpm / 300.0f) * 100.0f;
            LIMIT(current_value[i], -100.0f, 100.0f);
        }
        else
        {
            /* 位置式：读取编码器角度，转换为-180~180 */
            // TODO: 需要实现角度读取函数
            // current_value[i] = Encoder_GetAngle(i);
            current_value[i] = 0.0f;  // 临时占位
        }

        /* 2. PID计算 */
        float output = Motor_PID_Calculate(&motor_pid[i],
                                           target_value[i],
                                           current_value[i]);

        /* 3. 输出到电机 */
        Motor_Set(i, (int16_t)output);
    }
}

/* ===================== 定时器中断回调（需要在stm32f4xx_it.c中调用）===================== */
/*
 * 在 stm32f4xx_it.c 的 TIM6_DAC_IRQHandler 中添加：
 *
 * void TIM6_DAC_IRQHandler(void)
 * {
 *     HAL_TIM_IRQHandler(&htim6);
 * }
 *
 * 然后在 tim.c 的 HAL_TIM_PeriodElapsedCallback 中添加：
 *
 * void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * {
 *     if (htim->Instance == TIM6)
 *     {
 *         Motor_Control_Loop();
 *     }
 * }
 */
