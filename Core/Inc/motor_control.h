/*
 * motor_control.h - 电机PID闭环控制模块
 *
 * 提供两个电机的速度闭环控制接口
 */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#include "main.h"
#include "PID.h"

/* 电机数量 */
#define MOTOR_CONTROL_NUM 2

/* 控制周期（ms） */
#define CONTROL_PERIOD_MS 10

/**
 * @brief  初始化电机控制系统
 * @note   在main函数中调用一次，会自动配置定时器中断
 */
void Motor_Control_Init(void);

/**
 * @brief  设置电机目标值和PID参数
 * @param  motor_id: 电机ID (0-1)
 * @param  type: PID类型 (PID_TYPE_POSITION=0 位置式, PID_TYPE_INCREMENTAL=1 增量式)，负值表示不修改
 * @param  target: 目标值
 *                 - 增量式(速度控制): -100 ~ 100 (速度百分比)
 *                 - 位置式(角度控制): -180 ~ 180 (角度，单位：度)
 *                 - 负值表示不修改
 * @param  kp, ki, kd: PID系数，负值表示不修改
 * @param  max_output: 输出限幅，负值表示不修改
 */
void Motor_Control_SetPID(uint8_t motor_id, int8_t type, float target,
                          float kp, float ki, float kd, float max_output);

/**
 * @brief  获取电机当前值
 * @param  motor_id: 电机ID (0-1)
 * @return 当前值
 *         - 增量式: 当前速度百分比 (-100 ~ 100)
 *         - 位置式: 当前角度 (-180 ~ 180)
 */
float Motor_Control_GetSpeed(uint8_t motor_id);

/**
 * @brief  获取电机PID输出值
 * @param  motor_id: 电机ID (0-1)
 * @return PID输出值 (-100 ~ 100)
 */
float Motor_Control_GetOutput(uint8_t motor_id);

/**
 * @brief  停止所有电机
 */
void Motor_Control_StopAll(void);

/**
 * @brief  停止指定电机
 * @param  motor_id: 电机ID (0-1)
 */
void Motor_Control_Stop(uint8_t motor_id);

/**
 * @brief  PID控制循环（在定时器中断中调用）
 * @note   不要手动调用，由定时器中断自动调用
 */
void Motor_Control_Loop(void);

#endif /* __MOTOR_CONTROL_H */
