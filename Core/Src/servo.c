#include "servo.h"

/*
 * 舵机脉宽范围: 500μs ~ 2500μs
 * TIM1: 时钟 168MHz, Prescaler=1680-1, Period=2000-1
 * TIM3: 时钟 84MHz, Prescaler=840-1, Period=2000-1
 * 周期 = 84M / 840 / 2000 = 50Hz = 20ms
 * 每个计数 = 20ms / 2000 = 10μs
 *
 * 500μs  → Pulse = 50   → 0°
 * 2500μs → Pulse = 250  → 270°
 * 角度与Pulse线性关系: Pulse = 50 + angle * (250-50) / 270
 *                    Pulse = 50 + angle * 200 / 270
 *
 * STEER1=PC8 (TIM3_CH3), STEER2=PC9 (TIM3_CH4)
 * STEER3=PA8 (TIM1_CH1), STEER4=PA11 (TIM1_CH4)
 */

/* 舵机通道: 4个舵机 */
static const struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} servo_timer[SERVO_NUM] = {
    {&htim3, TIM_CHANNEL_3},  // Servo 0 - STEER1 = PC8
    {&htim3, TIM_CHANNEL_4},  // Servo 1 - STEER2 = PC9
    {&htim1, TIM_CHANNEL_1},  // Servo 2 - STEER3 = PA8
    {&htim1, TIM_CHANNEL_4},  // Servo 3 - STEER4 = PA11
};

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    // /* 初始回到 0° */
    // for (uint8_t i = 0; i < SERVO_NUM; i++)
    // {
    //     Servo_Set(i, 0);
    // }
}

void Servo_Set(uint8_t servo_id, float angle)
{
    if (servo_id >= SERVO_NUM)
        return;

    /* 限幅 0~270° */
    if (angle < 0)   angle = 0;
    if (angle > 270)  angle = 270;

    /* 角度转Pulse: 50 + angle * 200 / 270 */
    uint32_t pulse = (uint32_t)(50.0f + angle * 200.0f / 270.0f);

    __HAL_TIM_SET_COMPARE(servo_timer[servo_id].htim, servo_timer[servo_id].channel, pulse);
}
