/*
 * servo_tracker.c - K230 自瞄云台追踪模块
 *
 * 严格复制参考代码(F407空白模测试版)通信方式:
 *   - USART1 RXNE中断写入rx_buffer (在stm32f4xx_it.c)
 *   - 主循环检查new_data_flag
 *   - ParseK230Data: 帧尾0xFA触发, decode_coordinate解码
 *   - func_id: 0x00=X坐标, 0xFF=Y坐标
 *   - 位置式PID，直接输出舵机角度
 *
 * 输出:
 *   Servo 2 (PA8, TIM1_CH1) → X轴(水平)
 *   Servo 1 (PC9, TIM3_CH4) → Y轴(垂直)
 */

#include "servo_tracker.h"
#include "servo.h"
#include "PID.h"
#include "usart.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* ===================== 串口接收缓冲 (中断写入, 主循环读取) ===================== */

#define RX_BUFFER_SIZE  32

volatile uint8_t  st_rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t  st_rx_index = 0;
volatile uint8_t  st_new_data_flag = 0;
volatile uint8_t  st_has_target = 0;

static float tracking_target_x = 320.0f;
static float tracking_target_y = 240.0f;

/* ===================== decode_coordinate (复制参考代码) ===================== */

static int decode_coordinate(uint8_t h, uint8_t l, uint8_t z)
{
    if (h == 255 && l == 255) {
        return 510 + z;       /* 510~765 */
    } else if (h == 255 && l != 255) {
        return 255 + l;       /* 255~510 */
    } else {
        return h;             /* 0~255 */
    }
}

/* ===================== ParseK230Data (复制参考代码) ===================== */

void ServoTracker_ParseByte(uint8_t byte)
{
    /* ===== 调试: 每收到一个字节舵机摆一下 ===== */
    static uint8_t dbg_toggle = 0;
    dbg_toggle = !dbg_toggle;
    if (dbg_toggle) {
        Servo_Set(2, 100.0f);
    } else {
        Servo_Set(2, 170.0f);
    }
    /* ===== 调试结束 ===== */

    st_rx_buffer[st_rx_index++] = byte;

    /* 检测帧尾0xFA触发解析 */
    if (byte == 0xFA && st_rx_index >= 7) {
        uint8_t *data = (uint8_t *)st_rx_buffer;
        static int x_raw = 320;
        static int y_raw = 240;
        static uint8_t x_received = 0;
        static uint8_t y_received = 0;

        if (data[0] == 0x55 && data[1] == 0xAA) {
            if (data[2] == 0x00) {
                x_raw = decode_coordinate(data[3], data[4], data[5]);
                x_received = 1;
                tracking_target_x = (float)x_raw;
            }
            else if (data[2] == 0xFF) {
                y_raw = decode_coordinate(data[3], data[4], data[5]);
                y_received = 1;
                tracking_target_y = (float)y_raw;
            }

            if (x_received && y_received) {
                st_new_data_flag = 1;
                st_has_target = 1;
                x_received = 0;
                y_received = 0;
            }
        }
        st_rx_index = 0;
    }
    else if (st_rx_index >= RX_BUFFER_SIZE) {
        st_rx_index = 0;
    }
}

/* ===================== 内部变量 ===================== */

static ServoTracker tracker;

/* ===================== 公开接口 ===================== */

void ServoTracker_Init(void)
{
    memset(&tracker, 0, sizeof(tracker));
    st_rx_index = 0;
    st_new_data_flag = 0;
    st_has_target = 0;

    tracker.servo_x_angle = TRACKER_X_ANGLE_INIT;
    tracker.servo_y_angle = TRACKER_Y_ANGLE_INIT;
    tracker.data_valid = 0;
    tracker.enabled = 1;

    Servo_Set(2, tracker.servo_x_angle);
    Servo_Set(1, tracker.servo_y_angle);
}

void ServoTracker_Enable(uint8_t enable)
{
    tracker.enabled = enable;
}

void ServoTracker_Update(void)
{
    /* ===== 调试: 有数据来就让舵机来回摆动 ===== */
    if (st_new_data_flag && tracker.enabled) {
        st_new_data_flag = 0;

        static uint8_t toggle = 0;
        toggle = !toggle;
        if (toggle) {
            Servo_Set(2, 100.0f);  /* 摆到100° */
            Servo_Set(1, 100.0f);
        } else {
            Servo_Set(2, 170.0f);  /* 摆到170° */
            Servo_Set(1, 170.0f);
        }

        tracker.data_valid = 1;
        tracker.last_rx_tick = HAL_GetTick();
    }

    /* 超时检测 */
    if (tracker.data_valid &&
        (HAL_GetTick() - tracker.last_rx_tick > TRACKER_TIMEOUT_MS))
    {
        tracker.data_valid = 0;
        st_has_target = 0;
    }
}

const ServoTracker* ServoTracker_GetState(void)
{
    return &tracker;
}

uint8_t ServoTracker_HasTarget(void)
{
    return tracker.data_valid;
}
