/*
 * odometry.c - 通用里程计模块实现
 *
 * 支持底盘类型:
 *   - 两轮差速 (DIFF_2WD)
 *   - 四轮差速 (DIFF_4WD)
 *   - 三轮全向 (OMNI_3_WHEEL)
 *   - 四轮麦克纳姆 (MECANUM_4_WHEEL)
 *
 * 坐标系约定 (右手坐标系):
 *   - X 轴: 机器人正前方
 *   - Y 轴: 机器人正左方
 *   - Heading: 绕 Z 轴逆时针为正 (弧度)
 *   - 世界坐标系下积分时以初始朝向为基准
 *
 * 运动学模型:
 *   差速底盘: v = (vL + vR) / 2,   ω = (vR - vL) / track_width
 *   全向底盘: 速度分解到各轮，逆运动学求解
 */

#include "odometry.h"
#include "encoder.h"
#include "motor_control.h"
#include <math.h>
#include <string.h>

/* ===================== 数学常量 ===================== */

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define TWO_PI (2.0f * PI)
#define HALF_PI (PI / 2.0f)

/* ===================== 内部辅助函数 ===================== */

/*
 * normalize_angle - 将角度归一化到 [-PI, PI)
 */
static float normalize_angle(float angle)
{
    while (angle >= PI)  angle -= TWO_PI;
    while (angle < -PI)  angle += TWO_PI;
    return angle;
}

/*
 * encoder_to_mps - 编码器脉冲数转米/秒
 *
 * @param pulses   : 编码器脉冲增量
 * @param dt_s     : 时间步长 (秒)
 * @param encoder_ppr: 编码器 PPR
 * @param wheel_perimeter_mm: 轮子周长 (mm)
 * @return 轮子线速度 (m/s)
 */
static float encoder_to_mps(int32_t pulses, float dt_s,
                             float encoder_ppr, float wheel_perimeter_mm)
{
    if (dt_s <= 0.0f) return 0.0f;
    // 脉冲数 / PPR = 轮子转动的圈数
    // 圈数 * 周长 = 移动距离(m) / dt = 速度(m/s)
    float mps = (float)pulses / encoder_ppr * (wheel_perimeter_mm / 1000.0f) / dt_s;
    return mps;
}

/*
 * forward_diff_2wd - 两轮差速正运动学
 *
 * 输入: 左右轮速 (m/s), 轮距 (m)
 * 输出: 底盘线速度 (m/s), 角速度 (rad/s)
 */
static void forward_diff_2wd(float vL, float vR, float track_width,
                              float *v_out, float *omega_out)
{
    *v_out     = (vL + vR) * 0.5f;
    *omega_out = (vR - vL) / track_width;
}

/*
 * forward_omni_3wheel - 三轮全向正运动学
 *
 * 假设三个全向轮等距安装，角度分别为 0°、120°、240°
 * 求解底盘在 (X, Y, ω) 下的投影
 *
 * 逆运动学 (轮速 -> 底盘速度):
 *   ω_i = cos(α_i)·vx + sin(α_i)·vy + r·ω
 * 其中 α_i 为轮子朝向, r 为机器人半径
 *
 * 正运动学: 通过三个方程求解 vx, vy, ω
 */
static void forward_omni_3wheel(const float wheel_speeds[3],
                                 const float omega[3],
                                 float *vx_out, float *vy_out, float *omega_out)
{
    /* 三轮全向的几何约束:
     * 轮子均匀分布，安装角度相差 120° (2π/3)
     * 坐标变换矩阵 (伪逆):
     *
     *   [cosα0  sinα0  r]   [vx]   [v0]
     *   [cosα1  sinα1  r] × [vy] = [v1]
     *   [cosα2  sinα2  r]   [ω ]   [v2]
     *
     * 使用伪逆 A⁺ = (AᵀA)⁻¹ Aᵀ 求解
     */
    float r = 0.05f;  // 机器人半径 (m), 可调

    float ca[3], sa[3];
    for (int i = 0; i < 3; i++) {
        ca[i] = cosf(omega[i]);
        sa[i] = sinf(omega[i]);
    }

    /* AᵀA (3×3) */
    float AT_A[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        AT_A[0][0] += ca[i] * ca[i];  AT_A[0][1] += ca[i] * sa[i];  AT_A[0][2] += ca[i] * r;
        AT_A[1][0] += sa[i] * ca[i];  AT_A[1][1] += sa[i] * sa[i];  AT_A[1][2] += sa[i] * r;
        AT_A[2][0] += r * ca[i];       AT_A[2][1] += r * sa[i];       AT_A[2][2] += r * r;
    }

    /* (AᵀA)⁻¹ */
    float inv[3][3];
    // 对称矩阵求逆: 直接法
    float det = AT_A[0][0] * (AT_A[1][1]*AT_A[2][2] - AT_A[1][2]*AT_A[2][1])
              - AT_A[0][1] * (AT_A[1][0]*AT_A[2][2] - AT_A[1][2]*AT_A[2][0])
              + AT_A[0][2] * (AT_A[1][0]*AT_A[2][1] - AT_A[1][1]*AT_A[2][0]);
    if (fabsf(det) < 1e-6f) {
        *vx_out = 0; *vy_out = 0; *omega_out = 0;
        return;
    }
    float inv_det = 1.0f / det;
    inv[0][0] =  (AT_A[1][1]*AT_A[2][2] - AT_A[1][2]*AT_A[2][1]) * inv_det;
    inv[0][1] = -(AT_A[0][1]*AT_A[2][2] - AT_A[0][2]*AT_A[2][1]) * inv_det;
    inv[0][2] =  (AT_A[0][1]*AT_A[1][2] - AT_A[0][2]*AT_A[1][1]) * inv_det;
    inv[1][0] = -(AT_A[1][0]*AT_A[2][2] - AT_A[1][2]*AT_A[2][0]) * inv_det;
    inv[1][1] =  (AT_A[0][0]*AT_A[2][2] - AT_A[0][2]*AT_A[2][0]) * inv_det;
    inv[1][2] = -(AT_A[0][0]*AT_A[1][2] - AT_A[0][2]*AT_A[1][0]) * inv_det;
    inv[2][0] =  (AT_A[1][0]*AT_A[2][1] - AT_A[1][1]*AT_A[2][0]) * inv_det;
    inv[2][1] = -(AT_A[0][0]*AT_A[2][1] - AT_A[0][1]*AT_A[2][0]) * inv_det;
    inv[2][2] =  (AT_A[0][0]*AT_A[1][1] - AT_A[0][1]*AT_A[1][0]) * inv_det;

    /* x = (AᵀA)⁻¹ Aᵀ b */
    *vx_out = 0; *vy_out = 0; *omega_out = 0;
    for (int j = 0; j < 3; j++) {
        *vx_out     += inv[0][j] * ca[j] * wheel_speeds[j];
        *vy_out     += inv[1][j] * sa[j] * wheel_speeds[j];
        *omega_out  += inv[2][j] * r * wheel_speeds[j];
    }
}

/*
 * forward_mecanum_4wheel - 四轮麦克纳姆正运动学
 *
 * 麦克纳姆轮速度分解:
 *   v_wheel = vx·cos(α) + vy·sin(α) + track_width/2·ω·sgn(cos(α))
 *
 * 正运动学 (轮速 -> 底盘速度):
 *   使用伪逆求解 vx, vy, ω
 */
static void forward_mecanum_4wheel(const float wheel_speeds[4],
                                    const float omega[4],
                                    float track_width,
                                    float *vx_out, float *vy_out, float *omega_out)
{
    /* 构建 A 矩阵 (4×3):
     * 每行: [cos(α)  sin(α)  d·sign(cos(α))]
     * d = track_width/2 (或用轮子到中心距离的均值)
     */
    float d = track_width * 0.5f;
    if (d < 1e-6f) d = 0.1f;  // 防止除零

    float A[4][3] = {0};
    for (int i = 0; i < 4; i++) {
        float ca = cosf(omega[i]);
        float sa = sinf(omega[i]);
        float sign = (ca >= 0.0f) ? 1.0f : -1.0f;
        A[i][0] = ca;
        A[i][1] = sa;
        A[i][2] = d * sign;
    }

    /* AᵀA (3×3) */
    float ATA[3][3] = {0};
    for (int i = 0; i < 4; i++) {
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                ATA[r][c] += A[i][r] * A[i][c];
            }
        }
    }

    /* (AᵀA)⁻¹ */
    float inv[3][3];
    float det = ATA[0][0]*(ATA[1][1]*ATA[2][2]-ATA[1][2]*ATA[2][1])
              - ATA[0][1]*(ATA[1][0]*ATA[2][2]-ATA[1][2]*ATA[2][0])
              + ATA[0][2]*(ATA[1][0]*ATA[2][1]-ATA[1][1]*ATA[2][0]);
    if (fabsf(det) < 1e-6f) {
        *vx_out = 0; *vy_out = 0; *omega_out = 0;
        return;
    }
    float inv_det = 1.0f / det;
    inv[0][0] =  (ATA[1][1]*ATA[2][2] - ATA[1][2]*ATA[2][1]) * inv_det;
    inv[0][1] = -(ATA[0][1]*ATA[2][2] - ATA[0][2]*ATA[2][1]) * inv_det;
    inv[0][2] =  (ATA[0][1]*ATA[1][2] - ATA[0][2]*ATA[1][1]) * inv_det;
    inv[1][0] = -(ATA[1][0]*ATA[2][2] - ATA[1][2]*ATA[2][0]) * inv_det;
    inv[1][1] =  (ATA[0][0]*ATA[2][2] - ATA[0][2]*ATA[2][0]) * inv_det;
    inv[1][2] = -(ATA[0][0]*ATA[1][2] - ATA[0][2]*ATA[1][0]) * inv_det;
    inv[2][0] =  (ATA[1][0]*ATA[2][1] - ATA[1][1]*ATA[2][0]) * inv_det;
    inv[2][1] = -(ATA[0][0]*ATA[2][1] - ATA[0][1]*ATA[2][0]) * inv_det;
    inv[2][2] =  (ATA[0][0]*ATA[1][1] - ATA[0][1]*ATA[1][0]) * inv_det;

    /* x = (AᵀA)⁻¹ Aᵀ v */
    *vx_out = 0; *vy_out = 0; *omega_out = 0;
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 4; i++) {
            *vx_out    += inv[j][0] * A[i][j] * wheel_speeds[i];
            *vy_out    += inv[j][1] * A[i][j] * wheel_speeds[i];
            *omega_out += inv[j][2] * A[i][j] * wheel_speeds[i];
        }
    }
}

/* ===================== 积分 ===================== */

/*
 * integrate_position - 世界坐标系位置积分
 *
 * 在局部速度 (vx, vy) 和 heading 已知的情况下，
 * 投影到世界坐标系并积分
 */
static void integrate_position(OdomHandle *odom, float vx_body, float vy_body,
                               float v_linear, float v_angular, float dt_s)
{
    OdomState *s = &odom->state;

    if (v_angular < 1e-5f && v_angular > -1e-5f) {
        /* 近似直线运动 */
        float ds = v_linear * dt_s;
        s->x        += vx_body * dt_s;
        s->y        += vy_body * dt_s;
        s->heading  += v_angular * dt_s;
        s->total_distance += fabsf(ds);
    } else {
        /* 圆弧运动积分 */
        float d_heading = v_angular * dt_s;
        s->heading += d_heading;

        /* 曲率圆心偏移量 */
        float r = v_linear / v_angular;
        float delta_x = r * (sinf(s->heading) - sinf(s->heading - d_heading));
        float delta_y = r * (-cosf(s->heading) + cosf(s->heading - d_heading));

        s->x += delta_x;
        s->y += delta_y;
        s->total_distance += fabsf(v_linear * dt_s);
    }

    /* 归一化 heading */
    s->heading = normalize_angle(s->heading);
    s->total_angle += fabsf(v_angular * dt_s);
}

/* ===================== 卡尔曼滤波辅助 ===================== */

/* 卡尔曼滤波默认参数 */
#define ODOM_KF_Q_POS   0.05f   // 位置过程噪声
#define ODOM_KF_Q_VEL   0.5f    // 速度过程噪声
#define ODOM_KF_R_POS   2.0f    // 位置观测噪声
#define ODOM_KF_R_VEL   1.0f    // 速度观测噪声

/* ===================== 外部函数 ===================== */

void Odom_Init(OdomHandle *odom, ChassisType type, const OdomConfig *cfg)
{
    memset(odom, 0, sizeof(OdomHandle));

    odom->type = type;
    odom->config = *cfg;

    /* 自动计算轮子周长 */
    if (odom->config.wheel_perimeter < 1e-5f) {
        odom->config.wheel_perimeter = 2.0f * PI * odom->config.wheel_radius;
    }

    /* 自动计算编码器脉冲转 m/s 的系数
     * mps = pulses / encoder_ppr * perimeter_m / dt_s
     * 系数 = perimeter_m / encoder_ppr
     */
    odom->config.encoder_to_mps = (odom->config.wheel_perimeter / 1000.0f) / odom->config.encoder_ppr;

    /* 默认全向轮/麦克纳姆轮安装角度 */
    switch (type) {
        case CHASSIS_OMNI_3_WHEEL:
            /* 间隔 120° */
            odom->omega[0] = 0.0f;
            odom->omega[1] = TWO_PI / 3.0f;   /* 120° */
            odom->omega[2] = TWO_PI * 2.0f / 3.0f; /* 240° */
            odom->omega[3] = 0.0f;
            break;
        case CHASSIS_MECANUM_4_WHEEL:
            /* 标准麦克纳姆安装: X 型排列 */
            odom->omega[0] =  PI / 4.0f;   /*  45° 左前 */
            odom->omega[1] =  3.0f*PI/4.0f; /* 135° 左后 */
            odom->omega[2] = -PI / 4.0f;   /* -45° 右前 */
            odom->omega[3] = -3.0f*PI/4.0f; /*-135° 右后 */
            break;
        default:
            for (int i = 0; i < 4; i++) odom->omega[i] = 0.0f;
            break;
    }

    /* 里程计状态清零 */
    Odom_Reset(odom);

    odom->initialized = true;
}

void Odom_Reset(OdomHandle *odom)
{
    OdomState *s = &odom->state;
    s->x = 0.0f;
    s->y = 0.0f;
    s->heading = 0.0f;
    s->vx = 0.0f;
    s->vy = 0.0f;
    s->v_linear = 0.0f;
    s->v_angular = 0.0f;
    s->total_distance = 0.0f;
    s->total_angle = 0.0f;
    for (int i = 0; i < 4; i++) {
        s->wheel_speeds[i] = 0.0f;
    }
    for (int i = 0; i < ENCODER_NUM; i++) {
        odom->enc_delta[i] = 0;
    }
}

void Odom_Update(OdomHandle *odom, const float *gyro_yaw)
{
    if (!odom->initialized) return;

    float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;
    OdomConfig *cfg = &odom->config;
    OdomState *s = &odom->state;

    /* ===== 第1步: 读取编码器增量 ===== */
    /* 支持 2 或 4 电机: 用 ENCODER_NUM 决定循环次数 */
    for (int i = 0; i < ENCODER_NUM; i++) {
        odom->enc_delta[i] = Encoder_Read(i);
    }
    /* 清零计数器，确保下次 Encoder_Read 从 0 开始累计 */
    for (int i = 0; i < ENCODER_NUM; i++) {
        Encoder_Clear(i);
    }

    /* ===== 第2步: 编码器脉冲转轮速 (m/s) ===== */
    float ws[4] = {0};
    for (int i = 0; i < ENCODER_NUM; i++) {
        ws[i] = encoder_to_mps(odom->enc_delta[i], dt_s,
                                cfg->encoder_ppr, cfg->wheel_perimeter);
        s->wheel_speeds[i] = ws[i];
    }

    /* ===== 第3步: 运动学正解 ===== */
    float vx_body = 0.0f, vy_body = 0.0f;
    float v_linear = 0.0f, v_angular = 0.0f;

    switch (odom->type) {
#if ENCODER_NUM >= 4
        case CHASSIS_DIFF_4WD: {
            /* 平均前/后轮 */
            float vL = (ws[0] + ws[2]) * 0.5f;
            float vR = (ws[1] + ws[3]) * 0.5f;
            forward_diff_2wd(vL, vR, cfg->track_width / 1000.0f,
                              &v_linear, &v_angular);
            vx_body = v_linear;
            vy_body = 0.0f;
            break;
        }
        case CHASSIS_OMNI_3_WHEEL: {
            /* 只用 Motor0, 1, 2 */
            float ws3[3] = { ws[0], ws[1], ws[2] };
            float om3[3] = { odom->omega[0], odom->omega[1], odom->omega[2] };
            forward_omni_3wheel(ws3, om3, &vx_body, &vy_body, &v_angular);
            v_linear = sqrtf(vx_body*vx_body + vy_body*vy_body);
            break;
        }
        case CHASSIS_MECANUM_4_WHEEL: {
            forward_mecanum_4wheel(ws, odom->omega,
                                     cfg->track_width / 1000.0f,
                                     &vx_body, &vy_body, &v_angular);
            v_linear = sqrtf(vx_body*vx_body + vy_body*vy_body);
            break;
        }
        case CHASSIS_DIFF_2WD:
#else
        default:
#endif
        {
            /* Motor0=左轮, Motor1=右轮 (2WD 或 4电机不足时的默认) */
            float vL = ws[0];
            float vR = ws[1];
            forward_diff_2wd(vL, vR, cfg->track_width / 1000.0f,
                              &v_linear, &v_angular);
            vx_body = v_linear;
            vy_body = 0.0f;
            break;
        }
    }

    s->v_linear = v_linear;
    s->v_angular = v_angular;

    /* ===== 第4步: 陀螺仪 yaw 融合 (可选) ===== */
    if (odom->gyro_fusion_enabled && gyro_yaw != NULL) {
        float current_yaw = *gyro_yaw;
        if (odom->last_gyro_yaw > -1e6f) {
            float d_yaw = normalize_angle(current_yaw - odom->last_gyro_yaw);
            /* 使用陀螺仪 yaw 作为 heading，避免积分漂移 */
            s->heading = current_yaw;
            (void)d_yaw;  // 可用于计算 gyro_drift
        }
        odom->last_gyro_yaw = current_yaw;
    }

    /* ===== 第5步: 世界坐标系位置积分 ===== */
    integrate_position(odom, vx_body, vy_body, v_linear, v_angular, dt_s);

    /* 更新底盘世界速度 */
    float cos_h = cosf(s->heading);
    float sin_h = sinf(s->heading);
    s->vx = cos_h * vx_body - sin_h * vy_body;
    s->vy = sin_h * vx_body + cos_h * vy_body;

    /* ===== 第6步: 卡尔曼滤波 (可选) ===== */
    if (odom->kalman_enabled) {
        Kalman_Predict(&odom->kf_x);
        Kalman_Predict(&odom->kf_y);
        Kalman_Predict(&odom->kf_heading);
        Kalman_Predict(&odom->kf_vlinear);
        Kalman_Predict(&odom->kf_vangular);

        s->x         = Kalman_Update(&odom->kf_x, s->x);
        s->y         = Kalman_Update(&odom->kf_y, s->y);
        s->heading   = Kalman_Update(&odom->kf_heading, s->heading);
        s->v_linear  = Kalman_Update(&odom->kf_vlinear, s->v_linear);
        s->v_angular = Kalman_Update(&odom->kf_vangular, s->v_angular);
    }
}

const OdomState* Odom_GetState(const OdomHandle *odom)
{
    return &odom->state;
}

void Odom_EnableGyroFusion(OdomHandle *odom, bool enabled, float initial_yaw)
{
    odom->gyro_fusion_enabled = enabled;
    if (enabled) {
        odom->last_gyro_yaw = initial_yaw;
        odom->state.heading = initial_yaw;
    }
}

void Odom_EnableKalman(OdomHandle *odom, bool enabled, float dt)
{
    odom->kalman_enabled = enabled;
    if (enabled) {
        Kalman_InitParam(&odom->kf_x,         dt, ODOM_KF_Q_POS, ODOM_KF_Q_VEL, ODOM_KF_R_POS);
        Kalman_InitParam(&odom->kf_y,         dt, ODOM_KF_Q_POS, ODOM_KF_Q_VEL, ODOM_KF_R_POS);
        Kalman_InitParam(&odom->kf_heading,   dt, ODOM_KF_Q_POS, ODOM_KF_Q_VEL, ODOM_KF_R_POS);
        Kalman_InitParam(&odom->kf_vlinear,    dt, ODOM_KF_Q_VEL, ODOM_KF_Q_VEL, ODOM_KF_R_VEL);
        Kalman_InitParam(&odom->kf_vangular,  dt, ODOM_KF_Q_VEL, ODOM_KF_Q_VEL, ODOM_KF_R_VEL);
    }
}

void Odom_SetWheelAngle(OdomHandle *odom, uint8_t motor_id, float angle_rad)
{
    if (motor_id < 4) {
        odom->omega[motor_id] = angle_rad;
    }
}
