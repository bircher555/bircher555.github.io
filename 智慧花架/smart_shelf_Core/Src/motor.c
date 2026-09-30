#include "motor.h"

/*
 * Motor A: PWMA=PA2(TIM2_CH3), AIN1=PA6, AIN2=PA7
 * Motor B: PWMB=PA3(TIM2_CH4), BIN1=PA4, BIN2=PA5
 */

/* 每个电机对应的 TIM2 通道 */
static const uint32_t motor_channel[MOTOR_NUM] = {
    TIM_CHANNEL_3,  // Motor A - PA2
    TIM_CHANNEL_4   // Motor B - PA3
};

/* 方向引脚定义 */
typedef struct {
    GPIO_TypeDef *port1;
    uint16_t       pin1;
    GPIO_TypeDef *port2;
    uint16_t       pin2;
} Motor_PinDef;

static const Motor_PinDef motor_pins[MOTOR_NUM] = {
    {AIN1_GPIO_Port, AIN1_Pin, AIN2_GPIO_Port, AIN2_Pin},  // Motor A
    {BIN1_GPIO_Port, BIN1_Pin, BIN2_GPIO_Port, BIN2_Pin}   // Motor B
};

void Motor_Init(void)
{
    /* 启动 TIM2 两个通道的 PWM 输出 */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /* 初始全部停止 */
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        Motor_Set(i, 0);
    }
}

void Motor_Set(uint8_t motor_id, int16_t speed)
{
    if (motor_id >= MOTOR_NUM)
        return;

    /* 限幅 */
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    const Motor_PinDef *pin = &motor_pins[motor_id];

    if (speed > 0)
    {
        /* 正转: IN1=HIGH, IN2=LOW */
        HAL_GPIO_WritePin(pin->port1, pin->pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pin->port2, pin->pin2, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        /* 反转: IN1=LOW, IN2=HIGH */
        HAL_GPIO_WritePin(pin->port1, pin->pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(pin->port2, pin->pin2, GPIO_PIN_SET);
        speed = -speed;  /* 取绝对值给 PWM */
    }
    else
    {
        /* 刹车: IN1=HIGH, IN2=HIGH */
        HAL_GPIO_WritePin(pin->port1, pin->pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pin->port2, pin->pin2, GPIO_PIN_SET);
    }

    /* TIM2 Period=100, 所以 speed 直接作为 Pulse 值 */
    __HAL_TIM_SET_COMPARE(&htim2, motor_channel[motor_id], (uint32_t)speed);
}
