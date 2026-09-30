/*
 * ekf_pos_fusion.c - 里程计 + IMU EKF 位置融合实现
 *
 * 状态向量: X = [x, y, heading, vx, vy, omega]ᵀ  (6×1)
 *
 * EKF 预测步:
 *   使用编码器轮速积分预测状态
 *   P_ = F × P × Fᵀ + Q
 *
 * EKF 更新步 (三个观测源):
 *   A. 编码器速度: 更新 vx, vy, omega
 *   B. IMU 加速度: 世界坐标系加速度检测打滑，更新 vx, vy
 *   C. 陀螺仪 yaw : 更新 heading (消除积分漂移)
 *
 * 注意:
 *   - heading 和 yaw_gyro 在静止/匀速时等价
 *   - IMU 加速度世界坐标系投影基于 heading
 *   - 当前 IMU_Read_Raw() 返回全 0 时, IMU 更新步效果有限
 */

#include "ekf_pos_fusion.h"

/* ===================== EKF 内部变量 ===================== */

static Vector6 state;           // 状态向量 [x, y, heading, vx, vy, omega]
static Matrix6 P_mat;           // 误差协方差矩阵 (6×6)

/* 过程噪声协方差矩阵 Q (对角阵) */
static const float Q_diag[6] = {PF_Q_X, PF_Q_Y, PF_Q_HEADING,
                                PF_Q_VX, PF_Q_VY, PF_Q_OMEGA};

/* 观测噪声协方差矩阵 R (对角阵) */
static const float R_enc_diag[3] = {PF_R_ENC_VX, PF_R_ENC_VY, PF_R_ENC_OMEGA};
static const float R_imu_diag[2] = {PF_R_IMU_AX, PF_R_IMU_AY};
static const float R_yaw = PF_R_YAW;

/* 初始误差协方差 P (大方差表示初始不确定) */
#define P_INIT_DIAG  1000.0f

/* ===================== 输出变量 ===================== */

static PosFusionState fusion_state;
float odom_fusion_vx_enc = 0.0f;
float odom_fusion_vy_enc = 0.0f;
float odom_fusion_omega_enc = 0.0f;

/* ===================== 数学常量 ===================== */

#ifndef PI
#define PI 3.14159265358979323846f
#endif
#define TWO_PI (2.0f * PI)

/* ===================== 内部辅助 ===================== */

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
 * build_F_matrix - 构建状态转移矩阵 F (6×6)
 *
 * 状态转移方程:
 *   x      += (vx*cos(h) - vy*sin(h)) * dt
 *   y      += (vx*sin(h) + vy*cos(h)) * dt
 *   heading += omega * dt
 *   vx     ←  vx        (匀速模型)
 *   vy     ←  vy
 *   omega  ←  omega
 */
static void build_F_matrix(Matrix6 *F, float dt)
{
    Matrix6_Init(F, 6, 6);

    float h = state.data[2];
    float vx = state.data[3];
    float vy = state.data[4];

    float sin_h = sinf(h);
    float cos_h = cosf(h);
    float dt_vx = vx * dt;
    float dt_vy = vy * dt;

    /* x 行 */
    F->data[0][2] = (-dt_vx * cos_h - dt_vy * sin_h);  /* ∂x/∂heading */
    F->data[0][3] = cos_h * dt;                         /* ∂x/∂vx      */
    F->data[0][4] = -sin_h * dt;                        /* ∂x/∂vy      */

    /* y 行 */
    F->data[1][2] = (-dt_vx * sin_h + dt_vy * cos_h);  /* ∂y/∂heading */
    F->data[1][3] = sin_h * dt;                         /* ∂y/∂vx      */
    F->data[1][4] = cos_h * dt;                         /* ∂y/∂vy      */

    /* heading 行 */
    F->data[2][5] = dt;                                 /* ∂heading/∂omega */

    /* vx, vy, omega 行: 匀速模型 (对角 = 1) */
    F->data[3][3] = 1.0f;
    F->data[4][4] = 1.0f;
    F->data[5][5] = 1.0f;
}

/*
 * ekf_predict - EKF 预测步
 *
 * 1. 状态预测: 基于匀速模型积分位置和航向
 * 2. 协方差预测: P_ = F×P×Fᵀ + Q
 */
static void ekf_predict(float dt)
{
    /* ===== 步骤1: 状态预测 ===== */
    float h = state.data[2];
    float vx = state.data[3];
    float vy = state.data[4];
    float omega = state.data[5];

    float sin_h = sinf(h);
    float cos_h = cosf(h);

    /* 世界坐标系位置积分 */
    state.data[0] += (vx * cos_h - vy * sin_h) * dt;  /* x */
    state.data[1] += (vx * sin_h + vy * cos_h) * dt;  /* y */

    /* 航向积分 */
    state.data[2] += omega * dt;
    state.data[2] = normalize_angle(state.data[2]);
    /* vx, vy, omega 保持不变 (匀速模型) */

    /* ===== 步骤2: 协方差预测 ===== */
    Matrix6 F;
    build_F_matrix(&F, dt);
    Matrix6 FT = Matrix6_Transpose(&F);

    Matrix6 FP   = Matrix6_Multiply(&F, &P_mat);
    Matrix6 FPFt = Matrix6_Multiply(&FP, &FT);

    Matrix6 Q;
    Matrix6_Diagonal(&Q, Q_diag);
    Matrix6 P_pred = Matrix6_Add(&FPFt, &Q);
    P_mat = P_pred;
}

/*
 * ekf_update_encoder - 编码器速度观测更新
 *
 * 观测向量 Z_enc = [vx_enc, vy_enc, omega_enc]ᵀ
 * 观测矩阵 H_enc (3×6):
 *   H = [[0, 0, 0, 1, 0, 0],
 *        [0, 0, 0, 0, 1, 0],
 *        [0, 0, 0, 0, 0, 1]]
 */
static void ekf_update_encoder(float vx_enc, float vy_enc, float omega_enc)
{
    /* 观测向量 */
    float Z[3] = {vx_enc, vy_enc, omega_enc};

    /* 构建观测矩阵 H (3×6)
     * H[i][j] = 1 if j = i+3 else 0
     */
    float H[3][6] = {0};
    H[0][3] = 1.0f;
    H[1][4] = 1.0f;
    H[2][5] = 1.0f;

    /* 计算新息协方差 S = H×P×Hᵀ + R (3×3)
     * S[i][j] = sum_k,l H[i][k] * P[k][l] * H[j][l] + R[i][j]
     * 由于 H 的稀疏性: S[i][j] = P[i+3][j+3] + (i==j ? R_enc_diag[i] : 0)
     * 但必须计算完整的 S 矩阵，因为 P 不是对角
     */
    float S[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 6; k++) {
                for (int l = 0; l < 6; l++) {
                    S[i][j] += H[i][k] * P_mat.data[k][l] * H[j][l];
                }
            }
            if (i == j) {
                S[i][j] += R_enc_diag[i];
            }
        }
    }

    /* S⁻¹ */
    float invS[3][3];
    if (Matrix3_Inverse(S, invS)) return;  /* 奇异则跳过 */

    /* 计算卡尔曼增益 K = P × Hᵀ × S⁻¹ (6×3)
     * H 的稀疏性: H[j][k] = 1 仅当 k = j+3
     * 所以 (P×Hᵀ)[i][j] = P[i][j+3]
     * K = (P×Hᵀ) × S⁻¹
     */
    float K[6][3];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int m = 0; m < 3; m++) {
                K[i][j] += P_mat.data[i][m + 3] * invS[m][j];
            }
        }
    }

    /* 新息: innovation = Z - H×X = Z - [vx, vy, omega]ᵀ */
    float innovation[3];
    for (int i = 0; i < 3; i++) {
        innovation[i] = Z[i] - state.data[i + 3];
    }

    /* 状态更新: X = X + K × innovation */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            state.data[i] += K[i][j] * innovation[j];
        }
    }

    /* 协方差更新: Joseph形式 P = (I - K×H) × P × (I - K×H)ᵀ + K×R×Kᵀ
     * 数值稳定且保持对称性
     */
    /* K×H (6×6) */
    float KH[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    /* I - K×H */
    float I_minus_KH[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            I_minus_KH[i][j] = (i == j) ? 1.0f - KH[i][j] : -KH[i][j];
        }
    }

    /* (I - K×H) × P */
    float temp[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                temp[i][j] += I_minus_KH[i][k] * P_mat.data[k][j];
            }
        }
    }

    /* (I - K×H) × P × (I - K×H)ᵀ */
    float P_new[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                P_new[i][j] += temp[i][k] * I_minus_KH[j][k];
            }
        }
    }

    /* + K×R×Kᵀ */
    float KRK[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int m = 0; m < 3; m++) {
                KRK[i][j] += K[i][m] * R_enc_diag[m] * K[j][m];
            }
        }
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_new[i][j] += KRK[i][j];
        }
    }

    /* 更新协方差矩阵 */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_mat.data[i][j] = P_new[i][j];
        }
    }

    /* 归一化 heading */
    state.data[2] = normalize_angle(state.data[2]);
}

/*
 * ekf_update_imu_accel - IMU 加速度观测更新 (打滑检测)
 *
 * 原理: 匀速运动时世界坐标系加速度应接近 0
 *       如果 IMU 测到非零加速度 → 速度在变化 → 修正 vx/vy
 *
 * 观测向量 Z_imu = [acc_x_world, acc_y_world] (世界坐标系加速度)
 * 期望值 H×X     = [0, 0] (匀速时世界加速度为 0)
 *
 * 状态向量: X = [x, y, heading, vx, vy, omega]
 * 观测矩阵 H_imu (2×6): H[0][3]=cos(heading), H[0][4]=-sin(heading),
 *                       H[1][3]=sin(heading), H[1][4]=cos(heading)
 * 新息协方差 S = H×P×Hᵀ + R
 * 卡尔曼增益 K = P×Hᵀ×S⁻¹
 */
static void ekf_update_imu_accel(float acc_x_body, float acc_y_body)
{
    /* 将加速度从车身坐标系投影到世界坐标系 */
    float h = state.data[2];
    float sin_h = sinf(h);
    float cos_h = cosf(h);

    /* 世界坐标系加速度 (忽略重力影响，纯平动加速度) */
    float acc_x_world = acc_x_body * cos_h - acc_y_body * sin_h;
    float acc_y_world = acc_x_body * sin_h + acc_y_body * cos_h;

    /* 新息: 实际加速度与期望(0)的差值 */
    float innovation[2] = {acc_x_world, acc_y_world};

    /* 构建观测矩阵 H (2×6)
     * H[0][3] = cos(heading), H[0][4] = -sin(heading)
     * H[1][3] = sin(heading), H[1][4] = cos(heading)
     */
    float H[2][6] = {0};
    H[0][3] = cos_h;
    H[0][4] = -sin_h;
    H[1][3] = sin_h;
    H[1][4] = cos_h;

    /* 计算新息协方差 S = H×P×Hᵀ + R (2×2) */
    /* S_ij = sum_k H[i][k] * P[k][l] * H[j][l], summed over k and l */
    float S[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 6; k++) {
                for (int l = 0; l < 6; l++) {
                    S[i][j] += H[i][k] * P_mat.data[k][l] * H[j][l];
                }
            }
        }
    }
    /* 加上观测噪声 R */
    S[0][0] += R_imu_diag[0];
    S[1][1] += R_imu_diag[1];

    /* S⁻¹ */
    float invS[2][2];
    if (Matrix2_Inverse(S, invS)) return;

    /* 计算卡尔曼增益 K = P×Hᵀ×S⁻¹ (6×2) */
    /* K[i][j] = sum_k P[i][k] * H[j][k] * invS[j][j] */
    float K[6][2];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 2; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 6; k++) {
                K[i][j] += P_mat.data[i][k] * H[j][k];
            }
            K[i][j] *= invS[j][j];
        }
    }

    /* 状态更新: X = X + K × innovation */
    for (int i = 0; i < 6; i++) {
        state.data[i] += K[i][0] * innovation[0] + K[i][1] * innovation[1];
    }

    /* 协方差更新: Joseph形式 P = (I - K×H) × P × (I - K×H)ᵀ + K×R×Kᵀ
     * 数值稳定且保持对称性
     */
    float KH[6][6] = {0};  /* K×H (6×6) */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 2; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    /* I - K×H */
    float I_minus_KH[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            I_minus_KH[i][j] = (i == j) ? 1.0f - KH[i][j] : -KH[i][j];
        }
    }

    /* (I - K×H) × P */
    float temp[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                temp[i][j] += I_minus_KH[i][k] * P_mat.data[k][j];
            }
        }
    }

    /* (I - K×H) × P × (I - K×H)ᵀ */
    float P_new[6][6] = {0};
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 6; k++) {
                P_new[i][j] += temp[i][k] * I_minus_KH[j][k];
            }
        }
    }

    /* + K×R×Kᵀ */
    float KRK[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            KRK[i][j] = K[3 + i][0] * R_imu_diag[0] * K[3 + j][0] +
                        K[3 + i][1] * R_imu_diag[1] * K[3 + j][1];
        }
    }
    /* 添加到 P_new 对应的位置 */
    P_new[3][3] += KRK[0][0];
    P_new[3][4] += KRK[0][1];
    P_new[4][3] += KRK[1][0];
    P_new[4][4] += KRK[1][1];

    /* 更新协方差矩阵 */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_mat.data[i][j] = P_new[i][j];
        }
    }
}

/*
 * ekf_update_yaw - 陀螺仪 yaw 观测更新 (航向修正)
 *
 * 用 EKF 的 yaw 角修正里程计积分的 heading 漂移
 *
 * 观测: yaw_gyro (rad)
 * 期望: heading
 * 观测矩阵 H_yaw (1×6): H[0][2] = 1
 */
static void ekf_update_yaw(float yaw_gyro)
{
    /* 新息: yaw_gyro 和 heading 的差值 */
    float innovation = normalize_angle(yaw_gyro - state.data[2]);

    /* 新息协方差 S = P[2][2] + R_yaw (标量) */
    float S = P_mat.data[2][2] + R_yaw;

    /* 卡尔曼增益 K = P×Hᵀ/S (6×1) */
    /* H[0][2] = 1, 所以 K[i] = P[i][2] / S */
    float K[6];
    for (int i = 0; i < 6; i++) {
        K[i] = P_mat.data[i][2] / S;
    }

    /* 限制增益防止跳变 */
    if (K[2] > 0.8f) K[2] = 0.8f;
    if (K[2] < 0.01f) K[2] = 0.01f;
    /* 重新计算其他 K 值，保持比例 */
    for (int i = 0; i < 6; i++) {
        K[i] = P_mat.data[i][2] / S;
    }
    if (K[2] > 0.8f) {
        float scale = 0.8f / K[2];
        for (int i = 0; i < 6; i++) {
            K[i] *= scale;
        }
    }

    /* 状态更新 */
    state.data[2] = normalize_angle(state.data[2] + K[2] * innovation);

    /* 协方差更新: Joseph形式 P = (I - K×H) × P × (I - K×H)ᵀ + K×R×Kᵀ
     * H 是 1×6: H[0][2] = 1
     * 所以 KH[i][j] = K[i] * H[0][j] = K[i] if j==2 else 0
     */
    float P_new[6][6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            /* (I - KH) × P */
            float I_minus_KH_i = (i == 2) ? (1.0f - K[i]) : ((i == j) ? 1.0f : 0.0f);
            float sum_ij = 0.0f;
            for (int k = 0; k < 6; k++) {
                sum_ij += I_minus_KH_i * P_mat.data[k][j];
            }
            /* × (I - KH)ᵀ */
            for (int l = 0; l < 6; l++) {
                float I_minus_KH_l = (l == 2) ? (1.0f - K[l]) : ((l == j) ? 1.0f : 0.0f);
                P_new[i][l] += sum_ij * I_minus_KH_l;
            }
        }
    }

    /* + K×R×Kᵀ (只有 R_yaw 是标量) */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_new[i][j] += K[i] * R_yaw * K[j];
        }
    }

    /* 更新协方差矩阵 */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_mat.data[i][j] = P_new[i][j];
        }
    }

    /* 确保对称性和数值稳定性 */
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P_mat.data[i][j] = P_mat.data[j][i];
        }
        if (P_mat.data[i][i] < 0.001f) P_mat.data[i][i] = 0.001f;
    }
}

/* ===================== 外部函数 ===================== */

void PosFusion_Init(float x0, float y0, float heading0)
{
    /* 状态向量初始化 */
    state.data[0] = x0;
    state.data[1] = y0;
    state.data[2] = heading0;
    state.data[3] = 0.0f;  /* vx */
    state.data[4] = 0.0f;  /* vy */
    state.data[5] = 0.0f;  /* omega */

    /* 协方差矩阵初始化 (大方差) */
    Matrix6_Init(&P_mat, 6, 6);
    for (int i = 0; i < 6; i++) {
        P_mat.data[i][i] = P_INIT_DIAG;
    }

    /* 输出状态初始化 */
    fusion_state.x = x0;
    fusion_state.y = y0;
    fusion_state.heading = heading0;
    fusion_state.vx = 0.0f;
    fusion_state.vy = 0.0f;
    fusion_state.omega = 0.0f;
    fusion_state.v_linear = 0.0f;
}

void PosFusion_Update(float vx_enc, float vy_enc, float omega_enc,
                     float acc_x, float acc_y, float yaw_gyro, float dt)
{
    (void)dt;  /* dt 在预测步中暂不使用简化模型 */

    /* 步骤1: EKF 预测 */
    ekf_predict(dt);

    /* 步骤2: 编码器速度更新 (总是执行) */
    ekf_update_encoder(vx_enc, vy_enc, omega_enc);

    /* 步骤3: IMU 加速度更新 (检测打滑)
     * 仅当检测到非零加速度时才更新，避免静止时噪声干扰 */
    float acc_mag = fabsf(acc_x) + fabsf(acc_y);
    if (acc_mag > 0.05f) {  /* 阈值 0.05 m/s² */
        ekf_update_imu_accel(acc_x, acc_y);
    }

    /* 步骤4: 陀螺仪 yaw 更新 (修正 heading 漂移)
     * 仅当 yaw 明显偏离 heading 时更新 */
    float yaw_diff = fabsf(normalize_angle(yaw_gyro - state.data[2]));
    if (yaw_diff > 0.01f) {  /* 阈值 0.01 rad ≈ 0.57° */
        ekf_update_yaw(yaw_gyro);
    }

    /* 更新输出状态 */
    fusion_state.x = state.data[0];
    fusion_state.y = state.data[1];
    fusion_state.heading = state.data[2];
    fusion_state.vx = state.data[3];
    fusion_state.vy = state.data[4];
    fusion_state.omega = state.data[5];
    fusion_state.v_linear = sqrtf(state.data[3] * state.data[3] +
                                  state.data[4] * state.data[4]);
}

const PosFusionState* PosFusion_GetState(void)
{
    return &fusion_state;
}
