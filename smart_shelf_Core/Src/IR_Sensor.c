/*
 * 8路红外循迹模块 — USART2 (PA2 TX, PA3 RX)
 * 通信协议: 波特率 115200 / 8N1
 *
 * 控制命令 (STM32 → 传感器): $A,B,C#
 *   A=0 普通模式, A=1 校准
 *   B=0/1 不请求/请求模拟值
 *   C=0/1 不请求/请求数字值
 *
 * 传感器返回 (数字模式): $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
 *   每个 x = 0(白底) 或 1(黑线)
 *   解析方式: new_package[6 + i*5] 提取第 i 路的值
 */

#include "IR_Sensor.h"
#include "usart.h"
#include <string.h>

#define IR_PACKAGE_SIZE  100

static uint8_t  ir_rx_buf[IR_PACKAGE_SIZE];
static uint8_t  ir_new_pkg[IR_PACKAGE_SIZE];

/* 8路数字量结果: x1=bit0, x2=bit1 ... x8=bit7 */
static uint8_t  IR_Data_number[8];
static uint8_t  g_new_pkg_flag = 0;

/* --------------------- 初始化 --------------------- */

void IR_Sensor_Init(void)
{
    /* 启动 DMA 空闲中断接收，最大 100 字节 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ir_rx_buf, IR_PACKAGE_SIZE);

    /* 关闭半满中断，减少回调开销 */
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

    /* 发送控制命令: 普通模式 + 不请求模拟值 + 请求数字值 */
    uint8_t cmd[] = "$0,0,1#";
    HAL_UART_Transmit(&huart2, cmd, sizeof(cmd) - 1, 100);
}

/* --------------------- 读取接口 --------------------- */

/*
 * 读取单路传感器
 * ch: 1~8 (对应 x1~x8)
 * 返回: 0=白底, 1=黑线, 0xFF=无效数据
 */
uint8_t IR_Sensor_ReadChannel(uint8_t ch)
{
    if (ch < 1 || ch > 8) return 0xFF;

    uint8_t status;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (g_new_pkg_flag) {
        status = IR_Data_number[ch - 1];
    } else {
        status = 0xFF;
    }

    __set_PRIMASK(primask);
    return status;
}

/*
 * 一次性读取 8 路传感器
 * array: 外部分配的 uint8_t[8] 数组
 * array[0]=x1, array[1]=x2, ..., array[7]=x8
 */
void IR_Sensor_ReadAll(uint8_t *array)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (g_new_pkg_flag) {
        memcpy(array, IR_Data_number, 8);
        g_new_pkg_flag = 0;  // 消费数据
    } else {
        memset(array, 0xFF, 8);  // 无效数据填充 0xFF
    }

    __set_PRIMASK(primask);
}

/* --------------------- 内部: 帧解析 --------------------- */

/*
 * 逐字节处理，检测 $...# 帧
 * 帧格式: $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
 * 第 i 路值位于 new_package[6 + i*5]
 */
void IR_Sensor_ProcessByte(uint8_t byte)
{
    static uint8_t  rx_buf[IR_PACKAGE_SIZE];
    static uint8_t  step = 0;
    static uint8_t  started = 0;

    if (byte == '$') {
        started = 1;
        step = 0;
        rx_buf[step++] = byte;
    } else if (started) {
        rx_buf[step++] = byte;

        if (byte == '#') {
            /* 帧接收完成 */
            started = 0;
            step = 0;

            /* 复制到新缓冲区 */
            memcpy(ir_new_pkg, rx_buf, IR_PACKAGE_SIZE);
            g_new_pkg_flag = 1;

            /* 解析数字量: $D,x1:0,x2:0,...,x8:0# */
            /* 索引位置: $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0# */
            /*           0123456789012345678901234567890123456789012345678901 */
            /*           0         1         2         3         4    */
            if (ir_new_pkg[1] == 'D') {
                for (uint8_t i = 0; i < 8; i++) {
                    IR_Data_number[i] = ir_new_pkg[6 + i * 5] - '0';
                }
            }
            memset(rx_buf, 0, IR_PACKAGE_SIZE);
        }

        /* 数据异常保护 */
        if (step >= IR_PACKAGE_SIZE) {
            started = 0;
            step = 0;
            memset(rx_buf, 0, IR_PACKAGE_SIZE);
        }
    }
}

/* --------------------- UART 中断回调 --------------------- */

/*
 * UART DMA 接收完成回调 (空闲中断触发)
 * HAL_UARTEx_ReceiveToIdle_DMA 在一帧接收完毕后触发此回调
 *
 * 注意: UART2 → 循迹传感器, UART1 → 舵机追踪模块
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart2)
    {
        /* 循迹传感器数据 */
        for (uint16_t i = 0; i < Size; i++) {
            IR_Sensor_ProcessByte(ir_rx_buf[i]);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, ir_rx_buf, IR_PACKAGE_SIZE);
    }
    /* huart1 (K230) 的IDLE处理在 stm32f4xx_it.c 中，不在此回调 */
}
