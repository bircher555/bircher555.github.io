#include "encoder.h"

static TIM_HandleTypeDef *encoder_tim[ENCODER_NUM] = {
    &htim4,  // Motor A - Encoder A (PD12/PD13)
    &htim8   // Motor B - Encoder B (PC6/PC7)
};

/* 记录上次 Encoder_Read 返回的增量，供 Encoder_GetRPM 使用 */
static int32_t encoder_last_delta[ENCODER_NUM] = {0};

void Encoder_Init(void)
{
    for (uint8_t i = 0; i < ENCODER_NUM; i++)
    {
        HAL_TIM_Encoder_Start(encoder_tim[i], TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(encoder_tim[i], 0);
    }
}

int32_t Encoder_Read(uint8_t motor_id)
{
    if (motor_id >= ENCODER_NUM)
        return 0;

    TIM_HandleTypeDef *htim = encoder_tim[motor_id];

    // TI1 模式下，通过 CR1 DIR 位判断计数方向
    // DIR=0 向上计数(正转), DIR=1 向下计数(反转)
    int32_t count = (int32_t)__HAL_TIM_GET_COUNTER(htim);

    int32_t result;
    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
    {
        // 向下计数，认为是反转
        result = -count;
    }
    else
    {
        result = count;
    }

    encoder_last_delta[motor_id] = result;  // 记录本次增量
    __HAL_TIM_SET_COUNTER(htim, 0);         // 清零计数器
    return result;
}

/*
 * Encoder_Clear - 仅清零编码器计数器（不清零增量记录）
 *
 * 在 Odom_Update 读取增量后调用，确保下次 Encoder_Read 从 0 开始累计。
 * 注意: Encoder_Read 本身也会清零计数器，此函数用于在需要时额外清零。
 */
void Encoder_Clear(uint8_t motor_id)
{
    if (motor_id >= ENCODER_NUM)
        return;
    __HAL_TIM_SET_COUNTER(encoder_tim[motor_id], 0);
}

/*
 * Encoder_GetLast - 获取上次 Encoder_Read 返回的增量（不读取/不清零）
 *
 * 用于 Motor_Control_Loop 在同周期内复用 Odom_Update 已读取的增量，
 * 避免 Encoder_GetRPM 再次读取并清零计数器导致里程计数据丢失。
 */
int32_t Encoder_GetLast(uint8_t motor_id)
{
    if (motor_id >= ENCODER_NUM)
        return 0;
    return encoder_last_delta[motor_id];
}

float Encoder_GetRPM(uint8_t motor_id, uint16_t dt_ms)
{
    if (motor_id >= ENCODER_NUM || dt_ms == 0)
        return 0;

    /* 使用上次 Encoder_Read 返回的增量，不再次读取/清零计数器
     * 避免与 Odom_Update 竞争计数器清零 */
    int32_t pulses = Encoder_GetLast(motor_id);

    /*
     * TI1 模式单倍频，输出轴转一圈 = ENCODER_LINES * GEAR_RATIO 个脉冲
     * RPM = pulses / (编码器线数 * 减速比) / (dt_ms / 1000) * 60
     * 简化: RPM = pulses * 60000.0f / (ENCODER_LINES * GEAR_RATIO) / dt_ms
     */
    return (float)pulses * 60000.0f / (ENCODER_LINES * GEAR_RATIO) / (float)dt_ms;
}
