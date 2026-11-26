/**
 * ============================================================================
 * @file    ekf_localization.c
 * @brief   Hien thuc Odometry vi sai + EKF 3 trang thai [x, y, theta]
 *          chuan Joseph-form on dinh so hoc cho Cortex-M4F (float32)
 * ============================================================================
 */

#include "ekf_localization.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

float EKF_WrapPi(float angle_rad)
{
    if (!isfinite(angle_rad)) {
        return 0.0f;
    }
    if (angle_rad > ROBOT_PI_F || angle_rad < -ROBOT_PI_F) {
        float two_pi = 2.0f * ROBOT_PI_F;
        angle_rad = fmodf(angle_rad + ROBOT_PI_F, two_pi);
        if (angle_rad < 0.0f) {
            angle_rad += two_pi;
        }
        angle_rad -= ROBOT_PI_F;
    }
    return angle_rad;
}

static void ekf_symmetrize_and_clamp(EKF_Localizer_t *ekf)
{
    float p01 = 0.5f * (ekf->P[0][1] + ekf->P[1][0]);
    float p02 = 0.5f * (ekf->P[0][2] + ekf->P[2][0]);
    float p12 = 0.5f * (ekf->P[1][2] + ekf->P[2][1]);

    ekf->P[0][1] = ekf->P[1][0] = p01;
    ekf->P[0][2] = ekf->P[2][0] = p02;
    ekf->P[1][2] = ekf->P[2][1] = p12;

    if (ekf->P[0][0] < 1e-6f) ekf->P[0][0] = 1e-6f;
    if (ekf->P[1][1] < 1e-6f) ekf->P[1][1] = 1e-6f;
    if (ekf->P[2][2] < 1e-7f) ekf->P[2][2] = 1e-7f;
}

void EKF_Init(EKF_Localizer_t *ekf, float start_x_cm, float start_y_cm, float start_yaw_deg)
{
    if (ekf == NULL) {
        return;
    }
    memset(ekf, 0, sizeof(*ekf));

    if (!isfinite(start_x_cm))    start_x_cm    = 0.0f;
    if (!isfinite(start_y_cm))    start_y_cm    = 0.0f;
    if (!isfinite(start_yaw_deg)) start_yaw_deg = 0.0f;

    float yaw_rad = EKF_WrapPi(start_yaw_deg * ROBOT_DEG2RAD_F);
    ekf->pose.x_cm      = start_x_cm;
    ekf->pose.y_cm      = start_y_cm;
    ekf->pose.theta_rad = yaw_rad;
    ekf->pose.v_cms     = 0.0f;
    ekf->pose.omega_rads = 0.0f;

    ekf->odom_x_cm      = start_x_cm;
    ekf->odom_y_cm      = start_y_cm;
    ekf->odom_theta_rad = yaw_rad;

    /* Hiep phuong sai khoi tao nho tai vach xuat phat co dinh */
    ekf->P[0][0] = 0.25f;   /* std ~ 0.5 cm */
    ekf->P[1][1] = 0.25f;   /* std ~ 0.5 cm */
    ekf->P[2][2] = 0.0012f; /* std ~ 2.0 deg */
}

void EKF_ResetPose(EKF_Localizer_t *ekf, float x_cm, float y_cm, float yaw_deg,
                   bool imu_valid, float current_imu_yaw_deg)
{
    if (ekf == NULL) {
        return;
    }
    EKF_Init(ekf, x_cm, y_cm, yaw_deg);
    if (imu_valid && isfinite(current_imu_yaw_deg)) {
        float imu_rad = EKF_WrapPi(current_imu_yaw_deg * ROBOT_DEG2RAD_F);
        ekf->imu_yaw_offset_rad = EKF_WrapPi(ekf->pose.theta_rad - imu_rad);
        ekf->last_imu_yaw_rad   = imu_rad;
        ekf->imu_offset_locked  = true;
    }
}

void EKF_UpdateHeading(EKF_Localizer_t *ekf, float meas_yaw_rad, float var_rad2)
{
    if (ekf == NULL || !isfinite(meas_yaw_rad) || !isfinite(var_rad2) || var_rad2 <= 0.0f) {
        return;
    }

    /* H = [0, 0, 1] => S = P[2][2] + R */
    float s = ekf->P[2][2] + var_rad2;
    if (s <= 1e-9f) {
        return;
    }
    float inv_s = 1.0f / s;

    /* Kalman Gain K = P * H^T / S */
    float k0 = ekf->P[0][2] * inv_s;
    float k1 = ekf->P[1][2] * inv_s;
    float k2 = ekf->P[2][2] * inv_s;

    /* Sai so do luong (Innovation) da wrap [-pi, +pi] */
    float y_innov = EKF_WrapPi(meas_yaw_rad - ekf->pose.theta_rad);
    ekf->last_innovation_rad = y_innov;

    /* Cap nhat state (bao gom ca hieu chinh cheo x, y qua P[0][2], P[1][2]) */
    ekf->pose.x_cm      += k0 * y_innov;
    ekf->pose.y_cm      += k1 * y_innov;
    ekf->pose.theta_rad  = EKF_WrapPi(ekf->pose.theta_rad + k2 * y_innov);

    /* Joseph-form covariance update: P_new = (I - K H) P (I - K H)^T + K R K^T
     * Voi A = I - K H = [[1, 0, -k0], [0, 1, -k1], [0, 0, 1 - k2]] */
    float a22 = 1.0f - k2;
    float p[3][3];
    memcpy(p, ekf->P, sizeof(p));

    /* B = A * P */
    float b[3][3];
    for (int j = 0; j < 3; j++) {
        b[0][j] = p[0][j] - k0 * p[2][j];
        b[1][j] = p[1][j] - k1 * p[2][j];
        b[2][j] = a22 * p[2][j];
    }

    /* P_new = B * A^T + K * R * K^T */
    float k[3] = { k0, k1, k2 };
    for (int i = 0; i < 3; i++) {
        ekf->P[i][0] = b[i][0] - b[i][2] * k0 + k[i] * var_rad2 * k0;
        ekf->P[i][1] = b[i][1] - b[i][2] * k1 + k[i] * var_rad2 * k1;
        ekf->P[i][2] = b[i][2] * a22          + k[i] * var_rad2 * k2;
    }

    ekf_symmetrize_and_clamp(ekf);
}

void EKF_UpdatePosition(EKF_Localizer_t *ekf, float meas_x_cm, float meas_y_cm, float var_cm2)
{
    if (ekf == NULL || !isfinite(meas_x_cm) || !isfinite(meas_y_cm) ||
        !isfinite(var_cm2) || var_cm2 <= 0.0f) {
        return;
    }

    /* Cap nhat tuan tu truc X (H = [1,0,0]) roi truc Y (H = [0,1,0]) */
    const float meas[2] = { meas_x_cm, meas_y_cm };
    for (int axis = 0; axis < 2; axis++) {
        float s = ekf->P[axis][axis] + var_cm2;
        if (s <= 1e-9f) {
            continue;
        }
        float inv_s = 1.0f / s;
        float k[3] = {
            ekf->P[0][axis] * inv_s,
            ekf->P[1][axis] * inv_s,
            ekf->P[2][axis] * inv_s
        };

        float state_val = (axis == 0) ? ekf->pose.x_cm : ekf->pose.y_cm;
        float innov = meas[axis] - state_val;

        ekf->pose.x_cm      += k[0] * innov;
        ekf->pose.y_cm      += k[1] * innov;
        ekf->pose.theta_rad  = EKF_WrapPi(ekf->pose.theta_rad + k[2] * innov);

        float p[3][3];
        memcpy(p, ekf->P, sizeof(p));

        /* B = (I - K H) P */
        float b[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                b[i][j] = p[i][j] - k[i] * p[axis][j];
            }
        }
        /* P_new = B (I - K H)^T + K R K^T */
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ekf->P[i][j] = b[i][j] - b[i][axis] * k[j] + k[i] * var_cm2 * k[j];
            }
        }
        ekf_symmetrize_and_clamp(ekf);
    }
}

void EKF_Update(EKF_Localizer_t *ekf,
                int32_t delta_ticks_left,
                int32_t delta_ticks_right,
                float dt_s,
                bool imu_valid,
                float imu_yaw_deg,
                float imu_gz_rads)
{
    if (ekf == NULL) {
        return;
    }
    if (!isfinite(dt_s) || dt_s <= 1e-4f || dt_s > 0.5f) {
        dt_s = ROBOT_CTRL_DT_S;
    }
    if (!isfinite(imu_yaw_deg) || !isfinite(imu_gz_rads)) {
        imu_valid = false;
        imu_gz_rads = 0.0f;
    }

    ekf->update_count++;

    /* 0. Chan xung encoder bat thuong (bao ve neu hardware timer bi nhieu dot bien) */
    int32_t max_ticks = (int32_t)((5.0f * ROBOT_MAX_WHEEL_SPEED_CMS * dt_s) / ROBOT_CM_PER_TICK);
    if (max_ticks < 200) max_ticks = 200;
    if (delta_ticks_left  >  max_ticks) delta_ticks_left  =  max_ticks;
    if (delta_ticks_left  < -max_ticks) delta_ticks_left  = -max_ticks;
    if (delta_ticks_right >  max_ticks) delta_ticks_right =  max_ticks;
    if (delta_ticks_right < -max_ticks) delta_ticks_right = -max_ticks;

    /* 1. Tinh quang duong tinh tien & goc quay tu Encoder vi sai (2800 PPR) */
    float dl_cm = (float)delta_ticks_left  * ROBOT_CM_PER_TICK;
    float dr_cm = (float)delta_ticks_right * ROBOT_CM_PER_TICK;
    float ds_cm = 0.5f * (dl_cm + dr_cm);
    float dtheta_enc = (dr_cm - dl_cm) / ROBOT_TRACK_WIDTH_CM;

    ekf->pose.v_cms    = ds_cm / dt_s;
    ekf->omega_enc_rads = dtheta_enc / dt_s;

    /* 2. Cap nhat song song Dead Reckoning thuan encoder (de chan doan) */
    float odom_mid = ekf->odom_theta_rad + 0.5f * dtheta_enc;
    ekf->odom_x_cm      += ds_cm * cosf(odom_mid);
    ekf->odom_y_cm      += ds_cm * sinf(odom_mid);
    ekf->odom_theta_rad  = EKF_WrapPi(ekf->odom_theta_rad + dtheta_enc);

    /* 3. Kiem tra dung yen ZUPT (Zero-Velocity Update) chong troi khi cho */
    bool stationary = (delta_ticks_left == 0) && (delta_ticks_right == 0) &&
                      (!imu_valid || fabsf(imu_gz_rads) < EKF_ZUPT_GZ_THRESH_RADS);
    ekf->zupt_active = stationary;

    if (stationary) {
        ekf->pose.v_cms      = 0.0f;
        ekf->pose.omega_rads = 0.0f;
        ekf->wheel_slip_detected = false;
        ekf->imu_frozen_timer_s  = 0.0f;
        ekf->imu_frozen_enc_accum = 0.0f;
        if (imu_valid) {
            /* Lien tuc neo lai offset IMU trong luc dung yen de hap thu 100% troi nhiet
             * gyro (tranh nhay goc dot ngot khi xe bat dau chay sau 30s cho) */
            float imu_rad = EKF_WrapPi(imu_yaw_deg * ROBOT_DEG2RAD_F);
            ekf->imu_yaw_offset_rad = EKF_WrapPi(ekf->pose.theta_rad - imu_rad);
            ekf->last_imu_yaw_rad   = imu_rad;
            ekf->imu_offset_locked  = true;
        }
        return;
    }

    /* 4. Phat hien IMU bi dong bang khi xe dang xoay */
    float imu_rad = 0.0f;
    bool use_imu_meas = false;
    bool just_latched_frozen = false;
    if (imu_valid) {
        imu_rad = EKF_WrapPi(imu_yaw_deg * ROBOT_DEG2RAD_F);
        if (!ekf->imu_offset_locked) {
            ekf->imu_yaw_offset_rad   = EKF_WrapPi(ekf->pose.theta_rad - imu_rad);
            ekf->last_imu_yaw_rad     = imu_rad;
            ekf->imu_offset_locked    = true;
            ekf->imu_frozen_timer_s   = 0.0f;
            ekf->imu_frozen_enc_accum = 0.0f;
            ekf->imu_frozen           = false;
        } else {
            float diff_imu = fabsf(EKF_WrapPi(imu_rad - ekf->last_imu_yaw_rad));
            bool imu_stagnant = (diff_imu <= 0.0008f) && (fabsf(imu_gz_rads) < 0.03f);
            if (!imu_stagnant) {
                ekf->imu_frozen_timer_s   = 0.0f;
                ekf->imu_frozen_enc_accum = 0.0f;
                ekf->imu_frozen           = false;
                ekf->last_imu_yaw_rad     = imu_rad;
            } else if (fabsf(ekf->omega_enc_rads) > 0.35f) {
                if (ekf->imu_frozen_timer_s <= 0.0f && !ekf->imu_frozen) {
                    ekf->imu_frozen_theta_start = ekf->pose.theta_rad;
                    ekf->imu_frozen_enc_accum   = 0.0f;
                }
                ekf->imu_frozen_timer_s   += dt_s;
                ekf->imu_frozen_enc_accum += dtheta_enc;
                if (!ekf->imu_frozen && ekf->imu_frozen_timer_s >= EKF_IMU_FROZEN_TIMEOUT_S) {
                    ekf->imu_frozen     = true;
                    just_latched_frozen = true;
                }
            } else {
                ekf->imu_frozen_timer_s   = 0.0f;
                ekf->imu_frozen_enc_accum = 0.0f;
            }
        }
        use_imu_meas = !ekf->imu_frozen;
    } else {
        /* Neu IMU mat tam thoi, mo khoa offset de dong bo em khi phuc hoi */
        ekf->imu_offset_locked    = false;
        ekf->imu_frozen_timer_s   = 0.0f;
        ekf->imu_frozen_enc_accum = 0.0f;
    }

    /* 5. Phat hien truot banh (so sanh omega encoder vs omega gyro Z) */
    float q_theta_scale = 1.0f;
    if (use_imu_meas && fabsf(ekf->omega_enc_rads - imu_gz_rads) > EKF_SLIP_OMEGA_DIFF_RADS) {
        ekf->wheel_slip_detected = true;
        q_theta_scale = EKF_SLIP_Q_SCALE;
    } else {
        ekf->wheel_slip_detected = false;
    }

    /* 6. BUOC DU DOAN EKF (PREDICT) theo mo hinh dong hoc Midpoint RK2 */
    if (just_latched_frozen) {
        /* Phuc hoi goc quay encoder da bi IMU treo giu chat trong cua so phat hien 0.24s */
        ekf->pose.theta_rad = EKF_WrapPi(ekf->imu_frozen_theta_start +
                                         (ekf->imu_frozen_enc_accum - dtheta_enc));
    }
    float theta_mid = ekf->pose.theta_rad + 0.5f * dtheta_enc;
    float cos_m = cosf(theta_mid);
    float sin_m = sinf(theta_mid);

    ekf->pose.x_cm      += ds_cm * cos_m;
    ekf->pose.y_cm      += ds_cm * sin_m;
    ekf->pose.theta_rad  = EKF_WrapPi(ekf->pose.theta_rad + dtheta_enc);

    /* Jacobian trang thai F = [[1, 0, f02], [0, 1, f12], [0, 0, 1]] */
    float f02 = -ds_cm * sin_m;
    float f12 =  ds_cm * cos_m;

    /* Tinh P_pred = F * P * F^T */
    float p00 = ekf->P[0][0], p01 = ekf->P[0][1], p02 = ekf->P[0][2];
    float p11 = ekf->P[1][1], p12 = ekf->P[1][2], p22 = ekf->P[2][2];

    float fp00 = p00 + f02 * p02;
    float fp01 = p01 + f02 * p12;
    float fp02 = p02 + f02 * p22;
    float fp10 = p01 + f12 * p02;
    float fp11 = p11 + f12 * p12;
    float fp12 = p12 + f12 * p22;

    float np00 = fp00 + fp02 * f02;
    float np01 = fp01 + fp02 * f12;
    float np02 = fp02;
    float np11 = fp11 + fp12 * f12;
    float np12 = fp12;
    float np22 = p22;
    (void)fp10;

    /* Nhieu qua trinh phu thuoc chuyen dong: Q = G * diag(var_s, var_th) * G^T */
    float ds2 = ds_cm * ds_cm;
    float dt2 = dtheta_enc * dtheta_enc;
    float var_s  = EKF_ALPHA_SS * ds2 + EKF_ALPHA_ST * dt2 + EKF_Q_FLOOR_POS;
    float var_th = (EKF_ALPHA_TS * ds2 + EKF_ALPHA_TT * dt2 + EKF_Q_FLOOR_THETA) * q_theta_scale;

    float g00 = cos_m;
    float g10 = sin_m;
    float g01 = -0.5f * ds_cm * sin_m;
    float g11 =  0.5f * ds_cm * cos_m;

    ekf->P[0][0] = np00 + g00 * g00 * var_s + g01 * g01 * var_th;
    ekf->P[0][1] = np01 + g00 * g10 * var_s + g01 * g11 * var_th;
    ekf->P[0][2] = np02 + g01 * var_th;
    ekf->P[1][0] = ekf->P[0][1];
    ekf->P[1][1] = np11 + g10 * g10 * var_s + g11 * g11 * var_th;
    ekf->P[1][2] = np12 + g11 * var_th;
    ekf->P[2][0] = ekf->P[0][2];
    ekf->P[2][1] = ekf->P[1][2];
    ekf->P[2][2] = np22 + var_th;

    ekf_symmetrize_and_clamp(ekf);

    /* 7. BUOC CAP NHAT EKF (UPDATE) voi goc Yaw IMU 200Hz */
    if (use_imu_meas) {
        float meas_world_yaw = EKF_WrapPi(imu_rad + ekf->imu_yaw_offset_rad);
        EKF_UpdateHeading(ekf, meas_world_yaw, EKF_R_IMU_YAW_RAD2);
        ekf->pose.omega_rads = imu_gz_rads;
    } else {
        ekf->pose.omega_rads = ekf->omega_enc_rads;
        if (imu_valid && ekf->imu_frozen) {
            /* Dong bo lai offset khi IMU tinh day sau dong bang */
            ekf->imu_yaw_offset_rad = EKF_WrapPi(ekf->pose.theta_rad - imu_rad);
        }
    }
}

float EKF_GetYawDeg(const EKF_Localizer_t *ekf)
{
    if (ekf == NULL) {
        return 0.0f;
    }
    return ekf->pose.theta_rad * ROBOT_RAD2DEG_F;
}

RobotPose_t EKF_GetPose(const EKF_Localizer_t *ekf)
{
    RobotPose_t zero = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    if (ekf == NULL) {
        return zero;
    }
    return ekf->pose;
}
