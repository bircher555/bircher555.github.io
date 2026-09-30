/*
 * servo_tracker.h - K230 自瞄云台追踪模块
 *
 * 功能: 接收K230摄像头发送的目标坐标,
 *       通过PID控制两个舵机(云台)实时跟踪目标
 *
 * 协议(main_ai.py): 0x55, 0xAA, func_id, dataH, dataL, dataZ, 0xFA (7字节)
 *   func_id: 0xFF = X坐标, 0x00 = Y坐标
 *   坐标 = dataH + dataL + dataZ
 *   X和Y分两个包发送
 *
 * 图像分辨率: 640x480
 * 中心坐标: (320, 240)
 *
 * 接收方式: USART3轮询接收 (PD8=TX, PD9=RX), 波特率115200
 *
 * 舵机映射:
 *   Servo 2 (PA8, TIM1_CH1) → X轴(水平)
 *   Servo 1 (PC9, TIM3_CH4) → Y轴(垂直)
 */

#ifndef __SERVO_TRACKER_H
#define __SERVO_TRACKER_H

#include "main.h"
#include <stdint.h>

/* ===================== K230协议参数 ===================== */

#define TRACKER_IMAGE_W      640
#define TRACKER_IMAGE_H      480
#define TRACKER_CENTER_X     320.0f
#define TRACKER_CENTER_Y     240.0f

/* 数据有效超时(ms) */
#define TRACKER_TIMEOUT_MS   500

/* ===================== 舵机角度范围 ===================== */

#define TRACKER_X_ANGLE_MIN   60.0f
#define TRACKER_X_ANGLE_MAX   200.0f
#define TRACKER_X_ANGLE_INIT  135.0f

#define TRACKER_Y_ANGLE_MIN   30.0f
#define TRACKER_Y_ANGLE_MAX   180.0f
#define TRACKER_Y_ANGLE_INIT  135.0f

/* ===================== 追踪状态 ===================== */

typedef struct {
    int16_t raw_cx;
    int16_t raw_cy;
    uint8_t data_valid;
    uint32_t last_rx_tick;
    float servo_x_angle;
    float servo_y_angle;
    uint8_t enabled;
} ServoTracker;

/* ===================== 接口函数 ===================== */

void ServoTracker_Init(void);
void ServoTracker_Enable(uint8_t enable);
void ServoTracker_Update(void);
void ServoTracker_ParseByte(uint8_t byte);  /* 中断中调用 */
const ServoTracker* ServoTracker_GetState(void);
uint8_t ServoTracker_HasTarget(void);

#endif /* __SERVO_TRACKER_H */
