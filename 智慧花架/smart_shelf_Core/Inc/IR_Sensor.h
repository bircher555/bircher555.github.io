#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include "main.h"
#include "usart.h"
#include <string.h>

/* 8路传感器编号: x1=最左侧, x8=最右侧 */
/* 数字模式返回值: 0=白底, 1=黑线 */

void IR_Sensor_Init(void);

/* 读取第 ch 路传感器 (ch: 1~8), 返回 0 或 1 */
uint8_t IR_Sensor_ReadChannel(uint8_t ch);

/* 一次性读取 8 路，结果存入 array[8] (须 pre-allocated) */
void IR_Sensor_ReadAll(uint8_t *array);

#endif
