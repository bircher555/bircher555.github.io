#include "run.h"
#include "Emm_V5.h"
#include "main.h"
#include "servo.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

/*此为完整特定赛道的运动函数
 * 车一边走(驱动电机) 一边转(步进电机偏转舵机方向)
 */

/* 第1段: 直行 + 步进偏转 */
static void Run_Segment1(void)
{
    Servo_Set(3, 135);           /* 舵机回中 */
    Motor_Set(0, 20);            /* 左轮前进 */
    Motor_Set(1, 20);            /* 右轮前进 */
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 1, 400, 50, 700, 0, 0);
    HAL_Delay(800);
}

/* 第2段*/
static void Run_Segment2(void)
{
    Servo_Set(3, 220);
    Motor_Set(0, 20);
    Motor_Set(1, 20);
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 0, 13, 5, 2500, 0, 0);
    HAL_Delay(3300);
}

/* 第3段*/
static void Run_Segment3(void)
{
    Servo_Set(3, 220);
    Motor_Set(0, 20);
    Motor_Set(1, 20);
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 0, 15, 5, 5000, 0, 0);
    HAL_Delay(3500);
}

static void Run_Segment4(void)
{
    Servo_Set(3, 220);
    Motor_Set(0, 20);
    Motor_Set(1, 20);
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 0, 35, 10, 2200, 0, 0);
    HAL_Delay(2000);
}

static void Run_Segment5(void)
{
    Servo_Set(3, 130);
    Motor_Set(0, 20);
    Motor_Set(1, 20);
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 0, 36, 10, 1500, 0, 0);     /* 步进停止 */
    HAL_Delay(800);
}

static void Run_Segment6(void)
{
    Servo_Set(3, 220);
    Motor_Set(0, 20);
    Motor_Set(1, 20);
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 0, 18, 5, 2900, 0, 0);     /* 步进停止 */
    HAL_Delay(4000);
}

static void Run_Segment7(void)
{
    Servo_Set(3, 220);
    Motor_Set(0, 20);
    Motor_Set(1, 20);
    Emm_V5_Set_Channel(&Emm_Ch1);
    Emm_V5_Pos_Control(1, 0, 10, 5, 3000, 0, 0);     /* 步进停止 */
    HAL_Delay(5200);
}

void Run(void)
{
    Servo_Set(3, 130);           /* 舵机回中 */
    HAL_Delay(1000);
    Run_Segment1();
    Run_Segment2();
    Run_Segment3();
    Run_Segment4();
    Run_Segment5();
    Run_Segment6();
    Run_Segment7();
    Servo_Set(3, 135);           /* 舵机回中 */
    Motor_Set(0, 0);
    Motor_Set(1, 0);
    HAL_Delay(1000000);
}
