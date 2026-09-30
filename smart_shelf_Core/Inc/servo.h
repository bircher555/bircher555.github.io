#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "tim.h"

#define SERVO_NUM 4

void Servo_Init(void);
void Servo_Set(uint8_t servo_id, float angle);

#endif
