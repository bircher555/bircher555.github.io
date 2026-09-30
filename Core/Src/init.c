/*
 * init.c - 系统初始化模块
 *
 * 集中管理所有外设和模块的初始化及主循环
 */

#include "init.h"
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include "servo.h"
#include "servo_tracker.h"
#include "smart_shelf.h"
#include "Emm_V5.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern Emm_V5_Channel_t Emm_Ch1;
extern Emm_V5_Channel_t Emm_Ch2;

void System_Init(void)
{
    /* CubeMX 外设初始化 */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM8_Init();
    MX_SPI2_Init();
    MX_TIM6_Init();

    /* 电机初始化 */
    Motor_Init();

    /* 舵机初始化 */
    Servo_Init();

    /* 步进电机 (Emm_V5) 初始化 */
    Emm_Ch1.huart = &huart4;
    Emm_Ch2.huart = &huart5;
    Emm_V5_Init(&Emm_Ch1);
    Emm_V5_Init(&Emm_Ch2);
    HAL_Delay(500);

#if SMART_SHELF_ENABLE
    /* 智慧花架: 传感器 / 视觉 / 执行器 / 看门狗
     * 须放在最后: 看门狗启动后主循环要在 2s 内开始调用 SmartShelf_Loop() */
    SmartShelf_Init();
#else
    /* 舵机追踪模块初始化 (K230 → USART1) */
    ServoTracker_Init();
    ServoTracker_Enable(1);
#endif
}
