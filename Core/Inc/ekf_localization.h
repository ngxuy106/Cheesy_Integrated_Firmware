/**
 * ============================================================================
 * @file    ekf_localization.h
 * @brief   Dinh vi Odometry vi sai + Bo loc Kalman mo rong EKF 3 trang thai
 *          [x_cm, y_cm, theta_rad] ket hop Encoder 2800 PPR + MPU6050 Gyro Z
 * ============================================================================
 */

#ifndef EKF_LOCALIZATION_H
#define EKF_LOCALIZATION_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Trang thai EKF da loc (he toa do sa ban: cm, rad) */
    RobotPose_t pose;

    /* Ma tran hiep phuong sai 3x3: [x, y, theta] */
    float P[3][3];

    /* Dead reckoning thuan encoder (luu song song de so sanh / chan doan) */
    float odom_x_cm;
    float odom_y_cm;
    float odom_theta_rad;

    /* Dong bo goc IMU voi he toa do sa ban */
    float imu_yaw_offset_rad;
    bool  imu_offset_locked;
    float last_imu_yaw_rad;
    float imu_frozen_timer_s;
    float imu_frozen_theta_start;
    float imu_frozen_enc_accum;
    bool  imu_frozen;

    /* Chan doan & bo loc truot banh */
    float omega_enc_rads;
    float last_innovation_rad;
    bool  wheel_slip_detected;
    bool  zupt_active;
    uint32_t update_count;
} EKF_Localizer_t;

/* Khoi tao / Dat lai EKF ve toa do xuat phat (cm, cm, deg) */
void EKF_Init(EKF_Localizer_t *ekf, float start_x_cm, float start_y_cm, float start_yaw_deg);

/* Dat lai pose tuc thoi va dong bo lai offset IMU (giu goc sa ban chinh xac) */
void EKF_ResetPose(EKF_Localizer_t *ekf, float x_cm, float y_cm, float yaw_deg,
                   bool imu_valid, float current_imu_yaw_deg);

/* Buoc du doan (Encoder Odometry) + Cap nhat (IMU Yaw) moi chu ky dt_s */
void EKF_Update(EKF_Localizer_t *ekf,
                int32_t delta_ticks_left,
                int32_t delta_ticks_right,
                float dt_s,
                bool imu_valid,
                float imu_yaw_deg,
                float imu_gz_rads);

/* Cap nhat do luong goc huong tuy chon (rad, phuong sai rad^2) - Joseph form */
void EKF_UpdateHeading(EKF_Localizer_t *ekf, float meas_yaw_rad, float var_rad2);

/* Cap nhat do luong vi tri tuy chon (x_cm, y_cm, phuong sai cm^2) */
void EKF_UpdatePosition(EKF_Localizer_t *ekf, float meas_x_cm, float meas_y_cm, float var_cm2);

/* Tien ich chuan hoa goc ve [-pi, +pi] va lay goc theo do */
float EKF_WrapPi(float angle_rad);
float EKF_GetYawDeg(const EKF_Localizer_t *ekf);
RobotPose_t EKF_GetPose(const EKF_Localizer_t *ekf);

#ifdef __cplusplus
}
#endif

#endif /* EKF_LOCALIZATION_H */
