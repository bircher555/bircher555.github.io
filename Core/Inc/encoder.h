#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "tim.h"

#define ENCODER_NUM 2

/* 310编码电机参数 */
#define ENCODER_LINES    13   // 编码器线数
#define GEAR_RATIO       20   // 减速比
/* TI1模式单倍频，输出轴转一圈脉冲数 = 13 * 20 = 260 */

void Encoder_Init(void);
int32_t Encoder_Read(uint8_t motor_id);
int32_t Encoder_GetLast(uint8_t motor_id);       /* 获取上次 Encoder_Read 的增量（不读不清零） */
void    Encoder_Clear(uint8_t motor_id);          /* 仅清零计数器 */
float   Encoder_GetRPM(uint8_t motor_id, uint16_t dt_ms);
float   Encoder_GetAngle(uint8_t motor_id);  // 获取角度 -180~180
void    Encoder_ResetAngle(uint8_t motor_id); // 重置角度为0

#endif
