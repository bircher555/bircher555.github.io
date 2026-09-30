#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"

#define MOTOR_NUM 2

void Motor_Init(void);
void Motor_Set(uint8_t motor_id, int16_t speed);

#endif
