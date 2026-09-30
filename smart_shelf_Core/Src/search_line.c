#include "search_line.h"
#include "motor.h"
#include "servo.h"
#include "IR_Sensor.h"

/*
 * 8路红外循迹 — 最简逻辑
 * x1(最左) ~ x8(最右), 0=黑线, 1=白底
 * 转向舵机: Servo 3 (PA11), 中值 135°
 * 左偏 → 舵机右打, 右偏 → 舵机左打
 */

#define SERVO_CENTER  135
#define TURN_LEFT     75   /* 舵机右转角度 */
#define TURN_RIGHT    220   /* 舵机左转角度 */
#define SPEED         0    /* 电机速度 0~100 */

void SearchLine_Run(void)
{
    uint8_t x[8];
    IR_Sensor_ReadAll(x);

    if (x[0] == 0xFF) return;  /* 数据无效，跳过 */

    /* 右半边看到黑线 → 车偏右 → 舵机左打 */
    if (x[5] == 0 || x[6] == 0 || x[7] == 0) {
        Servo_Set(3, TURN_RIGHT);
    }
    /* 左半边看到黑线 → 车偏左 → 舵机右打 */
    else if (x[0] == 0 || x[1] == 0 || x[2] == 0) {
        Servo_Set(3, TURN_LEFT);
    }
    /* 中间看到黑线 → 直行 */
    else if (x[3] == 0 || x[4] == 0) {
        Servo_Set(3, SERVO_CENTER);
    }
    /* 都没看到 → 保持上一次方向继续走 */
    /* 中间看到 → 直行 */

    Motor_Set(0, SPEED);
    Motor_Set(1, SPEED);
}
