/*
 * odometry.h - 通用里程计模块
 *
 * 支持多种底盘类型:
 *   - 两轮差速 (DIFF_2WD)
 *   - 四轮差速 (DIFF_4WD)
 *   - 三轮全向 (OMNI_3_WHEEL)
 *   - 四轮麦克纳姆 (MECANUM_4_WHEEL)
 *
 * 功能:
 *   - 编码器脉冲 → 轮速 (m/s)
 *   - 运动学模型 → 底盘线速度/角速度
 *   - 位置积分 → 世界坐标系 (X, Y, Heading)
 *   - 可选: 陀螺仪 yaw 融合 (修正积分漂移)
 *   - 可选: 卡尔曼滤波平滑里程计输出
 *
 * 使用方法:
 *   1. Odom_Init(&odom, CHASSIS_MECANUM_4, &cfg);
 *   2. 在定时器中断中 (如50Hz):
 *        Odom_Update(&odom);
 *
 * 编码器通道映射 (与 encoder.h 一致):
 *   Motor 0 -> 左前轮 (或左轮)
 *   Motor 1 -> 右前轮 (或右轮)
 *   Motor 2 -> 左后轮
 *   Motor 3 -> 右后轮
 */

#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "kalman.h"

/* ===================== 底盘类型 ===================== */

typedef enum {
    CHASSIS_DIFF_2WD,       // 两轮差速 (左轮 Motor0, 右轮 Motor1)
    CHASSIS_DIFF_4WD,       // 四轮差速 (平均 Motor0&Motor1, Motor2&Motor3)
    CHASSIS_OMNI_3_WHEEL,   // 三轮全向 (Motor0/1/2, 间隔120°)
    CHASSIS_MECANUM_4_WHEEL  // 四轮麦克纳姆
} ChassisType;

/* ===================== 全向轮/麦克纳姆轮安装角度 ===================== */

/*
 * OMNI_3_WHEEL 安装角度 (轮子朝向, 相对于机器人正前方):
 *   Motor0:  0°   (正前方)
 *   Motor1: 120°  (左前方)
 *   Motor2: 240°  (右后方)
 *
 * MECANUM_4_WHEEL 安装角度 (轮子朝向):
 *   Motor0:  45°  (左前, 辊子朝向右下)
 *   Motor1: 135°  (左后, 辊子朝向右上)
 *   Motor2: -45°  (右前, 辊子朝向左下)
 *   Motor3: -135° (右后, 辊子朝向左上)
 *
 * 以上为默认配置，如有不同请在 Odom_Init 后修改 odom.omega[] 数组
 */

/* ===================== 物理参数配置 ===================== */

typedef struct {
    float wheel_radius;       // 轮子半径 (mm)
    float wheelbase;          // 前后轴距 (mm) — 两轮差速时填0
    float track_width;        // 左右轮距 (mm) — 差速底盘必须填
    float wheel_perimeter;    // 轮子周长 (mm, 自动计算: 2*pi*r)
    uint16_t encoder_ppr;     // 编码器 PPR (编码器线数 × 减速比)
    float encoder_to_mps;     // 编码器脉冲转 m/s 的系数 (自动计算)
} OdomConfig;

/* ===================== 里程计状态 ===================== */

typedef struct {
    /* --- 世界坐标系下的位姿 --- */
    float x;          // X 坐标 (m)
    float y;          // Y 坐标 (m)
    float heading;    // 航向角 (rad), 0=正北/X+, 逆时针为正

    /* --- 底盘速度 (世界坐标系) --- */
    float vx;         // X 方向线速度 (m/s)
    float vy;         // Y 方向线速度 (m/s)
    float v_linear;   // 合线速度 (m/s)
    float v_angular;  // 角速度 (rad/s)

    /* --- 轮速 (局部坐标系) --- */
    float wheel_speeds[4];  // 各轮线速度 (m/s)

    /* --- 累积行程 --- */
    float total_distance;  // 累计行驶距离 (m)
    float total_angle;     // 累计转向角度 (rad)
} OdomState;

/* ===================== 里程计主体 ===================== */

typedef struct {
    /* --- 配置 --- */
    ChassisType type;
    OdomConfig  config;

    /* --- 状态 --- */
    OdomState state;

    /* --- 积分用 --- */
    int32_t enc_delta[4];      // 本次编码器增量

    /* --- 全向轮/麦克纳姆轮安装角度 (rad) --- */
    float omega[4];           // 各轮朝向

    /* --- 陀螺仪 yaw 融合 (可选) --- */
    bool   gyro_fusion_enabled;
    float  last_gyro_yaw;      // 上次 yaw (rad)
    float  gyro_drift;         // 陀螺仪漂移补偿 (rad/s)

    /* --- 卡尔曼滤波 --- */
    bool   kalman_enabled;
    KalmanFilter kf_x;         // X 坐标滤波
    KalmanFilter kf_y;         // Y 坐标滤波
    KalmanFilter kf_heading;   // 航向角滤波
    KalmanFilter kf_vlinear;   // 速度滤波
    KalmanFilter kf_vangular;  // 角速度滤波

    /* --- 配置标记 --- */
    bool initialized;
} OdomHandle;

/* ===================== 函数声明 ===================== */

/*
 * Odom_Init - 里程计初始化
 *
 * @param odom   : OdomHandle 实例
 * @param type   : 底盘类型
 * @param cfg    : 物理参数 (wheel_radius, track_width, encoder_ppr 必填)
 *
 * 示例:
 *   OdomConfig cfg = {
 *       .wheel_radius  = 30.0f,   // mm
 *       .track_width   = 200.0f,  // mm
 *       .wheelbase     = 0.0f,
 *       .encoder_ppr   = 260,
 *   };
 *   Odom_Init(&odom, CHASSIS_DIFF_2WD, &cfg);
 */
void Odom_Init(OdomHandle *odom, ChassisType type, const OdomConfig *cfg);

/*
 * Odom_Reset - 重置里程计位置
 *
 * 将 X, Y, Heading, total_distance, total_angle 归零
 * 注意: 不会重置积分偏移量
 */
void Odom_Reset(OdomHandle *odom);

/*
 * Odom_Update - 里程计更新 (每次调用对应一个控制周期)
 *
 * 在定时器中断中调用，频率与控制周期一致 (如 50Hz = 20ms)
 *
 * @param odom       : OdomHandle 实例
 * @param gyro_yaw   : 陀螺仪 yaw (rad), 如果没有陀螺仪填 NULL
 *
 * 示例 (50Hz 定时器中断):
 *   void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *       if (htim->Instance == TIM5) {
 *           static float last_yaw = 0;
 *           // 读取陀螺仪 yaw
 *           float current_yaw = EKF_GetAngles()->yaw * (PI / 180.0f);
 *           Odom_Update(&odom, &current_yaw);
 *       }
 *   }
 */
void Odom_Update(OdomHandle *odom, const float *gyro_yaw);

/*
 * Odom_GetState - 获取里程计状态
 *
 * @return OdomState 指针
 *
 * 示例:
 *   const OdomState *s = Odom_GetState(&odom);
 *   printf("X=%.3f Y=%.3f H=%.1fdeg V=%.2fm/s\r\n",
 *          s->x, s->y, s->heading * 180.0f / PI, s->v_linear);
 */
const OdomState* Odom_GetState(const OdomHandle *odom);

/*
 * Odom_EnableGyroFusion - 启用/禁用陀螺仪 yaw 融合
 *
 * @param enabled : true=启用, false=禁用
 * @param initial_yaw : 融合起始的 yaw 值 (rad)
 *
 * 说明: 启用后使用陀螺仪 yaw 替代积分计算的 heading，
 *       可消除纯编码器积分的航向漂移
 */
void Odom_EnableGyroFusion(OdomHandle *odom, bool enabled, float initial_yaw);

/*
 * Odom_EnableKalman - 启用/禁用卡尔曼滤波
 *
 * @param enabled : true=启用, false=禁用
 * @param dt      : 滤波周期 (秒), 与 Odom_Update 调用周期一致
 *
 * 说明: 使用 kalman.c 中的滤波对 X, Y, Heading, v_linear, v_angular
 *       分别进行一阶卡尔曼滤波，平滑噪声
 */
void Odom_EnableKalman(OdomHandle *odom, bool enabled, float dt);

/*
 * Odom_SetWheelAngle - 设置全向轮/麦克纳姆轮的安装角度
 *
 * @param motor_id : 电机编号 (0-3)
 * @param angle_rad: 轮子朝向 (rad), 相对于机器人正前方
 *
 * 示例 (三轮全向):
 *   Odom_SetWheelAngle(&odom, 0, 0.0f);                  // 前
 *   Odom_SetWheelAngle(&odom, 1, 2.094f);               // 120° = 2π/3
 *   Odom_SetWheelAngle(&odom, 2, 4.189f);               // 240° = 4π/3
 */
void Odom_SetWheelAngle(OdomHandle *odom, uint8_t motor_id, float angle_rad);

#endif /* __ODOMETRY_H */
