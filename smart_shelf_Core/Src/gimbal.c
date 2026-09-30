/*
 * gimbal.c - 双步进差动云台控制模块
 *
 * 机械结构:
 *   两个步进电机都挂在UART4上，通过地址(addr)区分，1号在左，2号在右边
 *
 * 差动运动学:
 *   旋转(偏航):  两电机同方向旋转 → 云台整体偏转
 *   俯仰:       两电机反方向旋转 → 云台一端抬升另一端降低
 *
 * 控制接口:
 *   使用 Emm_V5 接口控制 UART4 上的两个步进电机 (addr=1 和 addr=2)
 *
 * 使用方法:
 *   1. Gimbal_Init() 初始化
 *   2. Gimbal_SetTarget(yaw, pitch) 设置目标角度 (位置模式)
 *   3. Gimbal_Loop() 在主循环中周期性调用
 *
 *   或实时速度控制:
 *   Gimbal_SpeedControl(yaw_vel, pitch_vel) (速度模式)
 */

#include "gimbal.h"
#include "Emm_V5.h"
#include <math.h>

/* ===================== 辅助宏 ===================== */
#define clamp(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/* ===================== 内部变量 ===================== */

/* 云台状态 */
static Gimbal_State gimbal = {
    .yaw   = GIMBAL_YAW_INIT,
    .pitch = GIMBAL_PITCH_INIT,
    .target_yaw   = GIMBAL_YAW_INIT,
    .target_pitch = GIMBAL_PITCH_INIT,
    .enabled = 0
};

/* 上次发送命令的时间 (用于限速) */
static uint32_t last_cmd_tick = 0;
static uint16_t cmd_interval_ms = 50;  /* 命令发送间隔 */

/* ===================== 内部函数 ===================== */

/*
 * deg_to_pulses - 角度转换为脉冲数
 * 公式: pulses = deg / GIMBAL_DEG_PER_PULSE
 */
static int32_t deg_to_pulses(float deg)
{
    return (int32_t)(deg / GIMBAL_DEG_PER_PULSE);
}

/*
 * clamp - 限幅
 */
static float clampf(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

/*
 * send_position_cmd - 发送位置控制命令
 *
 * 差动计算:
 *   left_pulses  = yaw_pulses - pitch_pulses   → 电机1 (addr=1, UART4)
 *   right_pulses = yaw_pulses + pitch_pulses   → 电机2 (addr=2, UART4)
 *
 * 实测方向基准:
 *   纯 pitch 上仰: 电机1 dir=1(逆时针), 电机2 dir=0(顺时针)
 *   即 pitch>0 时 left_pulses<0, right_pulses>0
 *
 * 两个电机都在 UART4 (Emm_Ch1) 上，通过地址区分。
 * 发完第一条命令后需等待 DMA 发送完成，防止 static cmd 被覆写。
 *
 * @param yaw_pulses:   旋转目标脉冲数
 * @param pitch_pulses: 俯仰目标脉冲数
 */
static void send_position_cmd(int32_t yaw_pulses, int32_t pitch_pulses)
{
    /* 差动计算 */
    int32_t left_pulses  = yaw_pulses - pitch_pulses;
    int32_t right_pulses = yaw_pulses + pitch_pulses;

    /* 两个电机都在 UART4 上，通过地址区分 */
    Emm_V5_Set_Channel(&Emm_Ch1);

    /* 电机1 (addr=1) */
    Emm_V5_Pos_Control(1,
                       (left_pulses >= 0) ? 0 : 1,
                       GIMBAL_MAX_SPEED,
                       GIMBAL_ACCEL,
                       (left_pulses >= 0) ? left_pulses : -left_pulses,
                       0, 0);

    HAL_Delay(10);  /* 等 DMA 发完再发下一条，防止 static cmd 被覆写 */

    /* 电机2 (addr=2) */
    Emm_V5_Pos_Control(2,
                       (right_pulses >= 0) ? 0 : 1,
                       GIMBAL_MAX_SPEED,
                       GIMBAL_ACCEL,
                       (right_pulses >= 0) ? right_pulses : -right_pulses,
                       0, 0);
}

/*
 * send_velocity_cmd - 发送速度控制命令
 *
 * 两个电机都在 UART4 (Emm_Ch1) 上，通过地址区分。
 * 发完第一条命令后需等待 DMA 发送完成，防止 static cmd 被覆写。
 *
 * @param yaw_vel:    旋转速度 (RPM)
 * @param pitch_vel:  俯仰速度 (RPM)
 */
static void send_velocity_cmd(int16_t yaw_vel, int16_t pitch_vel)
{
    /* 差动计算 */
    int16_t left_vel  = yaw_vel - pitch_vel;   /* 电机1 (addr=1) */
    int16_t right_vel = yaw_vel + pitch_vel;   /* 电机2 (addr=2) */

    /* 限幅 */
    left_vel  = clamp(left_vel,  -GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED);
    right_vel = clamp(right_vel, -GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED);

    if (left_vel == 0 && right_vel == 0) {
        /* 停止 */
        Emm_V5_Set_Channel(&Emm_Ch1);
        Emm_V5_Stop_Now(1, 0);
        HAL_Delay(10);
        Emm_V5_Stop_Now(2, 0);
        return;
    }

    /* 两个电机都在 UART4 上，通过地址区分 */
    Emm_V5_Set_Channel(&Emm_Ch1);

    /* 电机1 (addr=1) */
    if (left_vel != 0) {
        Emm_V5_Vel_Control(1,
                           (left_vel >= 0) ? 0 : 1,
                           (left_vel >= 0) ? left_vel : -left_vel,
                           GIMBAL_ACCEL, 0);
    }

    HAL_Delay(10);  /* 等 DMA 发完再发下一条 */

    /* 电机2 (addr=2) */
    if (right_vel != 0) {
        Emm_V5_Vel_Control(2,
                           (right_vel >= 0) ? 0 : 1,
                           (right_vel >= 0) ? right_vel : -right_vel,
                           GIMBAL_ACCEL, 0);
    }
}

/* ===================== 外部接口 ===================== */

void Gimbal_Init(void)
{
    /* 初始化状态 */
    gimbal.yaw   = GIMBAL_YAW_INIT;
    gimbal.pitch = GIMBAL_PITCH_INIT;
    gimbal.target_yaw   = GIMBAL_YAW_INIT;
    gimbal.target_pitch = GIMBAL_PITCH_INIT;
    gimbal.enabled = 1;

    last_cmd_tick = HAL_GetTick();

}

void Gimbal_SetTarget(float yaw, float pitch)
{
    /* 限幅 */
    gimbal.target_yaw   = clampf(yaw,   GIMBAL_YAW_MIN,   GIMBAL_YAW_MAX);
    gimbal.target_pitch = clampf(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
}

void Gimbal_SetYaw(float yaw)
{
    gimbal.target_yaw = clampf(yaw, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
}

void Gimbal_SetPitch(float pitch)
{
    gimbal.target_pitch = clampf(pitch, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
}

void Gimbal_SpeedControl(float yaw_speed, float pitch_speed)
{
    if (!gimbal.enabled) return;

    /* 转换为整数 RPM，限幅 */
    int16_t yaw_vel   = (int16_t)clampf(yaw_speed,   -GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED);
    int16_t pitch_vel = (int16_t)clampf(pitch_speed, -GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED);

    send_velocity_cmd(yaw_vel, pitch_vel);
}

void Gimbal_Stop(void)
{
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Stop_Now(1, 0);
    HAL_Delay(10);
    Emm_V5_Stop_Now(2, 0);
}

void Gimbal_Enable(uint8_t enable)
{
    gimbal.enabled = enable;
    if (!enable) {
        Gimbal_Stop();
    }
}

const Gimbal_State* Gimbal_GetState(void)
{
    return &gimbal;
}

void Gimbal_Loop(void)
{
    if (!gimbal.enabled) return;

    uint32_t now = HAL_GetTick();
    if (now - last_cmd_tick < cmd_interval_ms) return;

    /* 计算角度误差 */
    float yaw_err   = gimbal.target_yaw   - gimbal.yaw;
    float pitch_err = gimbal.target_pitch - gimbal.pitch;

    /* 误差小于阈值时认为到达目标 */
    if (fabsf(yaw_err) < 0.5f && fabsf(pitch_err) < 0.5f) {
        Gimbal_Stop();
        return;
    }

    /* 角度转换为脉冲 */
    int32_t yaw_pulses   = deg_to_pulses(yaw_err);
    int32_t pitch_pulses = deg_to_pulses(pitch_err);

    /* 发送位置命令 */
    send_position_cmd(yaw_pulses, pitch_pulses);

    last_cmd_tick = now;
}
