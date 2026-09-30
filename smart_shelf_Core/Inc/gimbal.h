/*
 * gimbal.h - 双步进差动云台控制模块
 *
 * 云台结构:
 *   两个步进电机都挂在UART4上，通过地址(addr)区分，1号在左，2号在右边
 *
 * 差动控制原理:
 *   旋转(偏航): 两电机同方向旋转
 *     left_speed  = yaw_speed - pitch_speed
 *     right_speed = yaw_speed + pitch_speed
 *
 *   俯仰: 两电机反方向旋转
 *     left_speed  = -pitch_speed
 *     right_speed =  pitch_speed
 *
 * 串口分配:
 *   两个电机都在 UART4 (Emm_Ch1) 上，通过地址区分
 *   电机1 (addr=1) → UART4
 *   电机2 (addr=2) → UART4
 *
 * 步进电机参数 (Emm_V5):
 *   速度单位: RPM
 *   位置单位: 脉冲数 (clk)
 */

#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ===================== 云台配置 ===================== */

/* 云台步进电机数量 */
#define GIMBAL_MOTOR_NUM    2

/* 云台运动参数 */
#define GIMBAL_MAX_SPEED    500     /* 最大速度 RPM */
#define GIMBAL_ACCEL        10      /* 加速度 */

/* 云台初始角度 (度) */
#define GIMBAL_YAW_INIT     0.0f    /* 旋转角度 */
#define GIMBAL_PITCH_INIT   0.0f    /* 俯仰角度 */

/* 云台角度限位 (度) */
#define GIMBAL_YAW_MIN      -180.0f
#define GIMBAL_YAW_MAX      180.0f
#define GIMBAL_PITCH_MIN    -90.0f
#define GIMBAL_PITCH_MAX    90.0f

/* 云台机械参数 */
#define GIMBAL_DEG_PER_PULSE   0.01125f   /* 每脉冲对应角度 (度) */
                                          /* 360度 / 32000步 = 0.01125度/脉冲 */

/* ===================== 云台状态 ===================== */

typedef struct {
    float yaw;           /* 当前旋转角度 (度) */
    float pitch;         /* 当前俯仰角度 (度) */
    float target_yaw;    /* 目标旋转角度 */
    float target_pitch;   /* 目标俯仰角度 */
    uint8_t enabled;     /* 使能标志 */
} Gimbal_State;

/* ===================== 接口函数 ===================== */

/**
 * @brief  初始化云台
 *         启动 UART4 DMA 接收，初始化云台状态
 */
void Gimbal_Init(void);

/**
 * @brief  设置云台目标角度
 * @param  yaw: 目标旋转角度 (度)
 * @param  pitch: 目标俯仰角度 (度)
 */
void Gimbal_SetTarget(float yaw, float pitch);

/**
 * @brief  设置云台旋转角度 (偏航)
 * @param  yaw: 目标旋转角度 (度)
 */
void Gimbal_SetYaw(float yaw);

/**
 * @brief  设置云台俯仰角度
 * @param  pitch: 目标俯仰角度 (度)
 */
void Gimbal_SetPitch(float pitch);

/**
 * @brief  云台速度控制 (实时控制)
 * @param  yaw_speed: 旋转速度 (RPM, 正=顺时针)
 * @param  pitch_speed: 俯仰速度 (RPM, 正=抬头)
 */
void Gimbal_SpeedControl(float yaw_speed, float pitch_speed);

/**
 * @brief  停止云台
 */
void Gimbal_Stop(void);

/**
 * @brief  使能/禁用云台
 * @param  enable: 1=使能, 0=禁用
 */
void Gimbal_Enable(uint8_t enable);

/**
 * @brief  获取云台当前状态
 * @return Gimbal_State 指针
 */
const Gimbal_State* Gimbal_GetState(void);

/**
 * @brief  云台控制循环 (在主循环或定时器中调用)
 *         根据目标角度和当前角度，计算并发送步进电机命令
 */
void Gimbal_Loop(void);

#endif /* __GIMBAL_H */