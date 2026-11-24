/**
 * ============================================================================
 * @file    test_ekf_odometry.c
 * @brief   Unit tests chuyen sau cho Encoder Odometry + EKF 3 trang thai
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>
#include "robot_config.h"
#include "ekf_localization.h"

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_TRUE(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("[FAIL] Line %d: %s\n", __LINE__, msg); } \
} while (0)

#define EXPECT_NEAR(val, target, tol, msg) do { \
    float _v = (val), _t = (target), _tol = (tol); \
    if (isfinite(_v) && fabsf(_v - _t) <= _tol) { g_pass++; } \
    else { g_fail++; printf("[FAIL] Line %d: %s (got %.4f, expected %.4f +/- %.4f)\n", \
                            __LINE__, msg, _v, _t, _tol); } \
} while (0)

static bool is_cov_valid(const EKF_Localizer_t *ekf)
{
    for (int i = 0; i < 3; i++) {
        if (!isfinite(ekf->P[i][i]) || ekf->P[i][i] <= 0.0f) return false;
        for (int j = 0; j < 3; j++) {
            if (!isfinite(ekf->P[i][j])) return false;
            if (fabsf(ekf->P[i][j] - ekf->P[j][i]) > 1e-5f) return false;
        }
    }
    return true;
}

static void test_straight_encoder_odometry(void)
{
    EKF_Localizer_t ekf;
    EKF_Init(&ekf, 30.0f, -6.0f, 90.0f);

    /* 1 vong banh = 2800 xung = pi * 4.5 = 14.137167 cm (chay trong 50 buoc) */
    for (int i = 0; i < 50; i++) {
        EKF_Update(&ekf, 56, 56, 0.02f, false, 0.0f, 0.0f);
    }

    float expected_dy = ROBOT_PI_F * ROBOT_WHEEL_DIAM_CM;
    EXPECT_NEAR(ekf.pose.x_cm, 30.0f, 1e-3f, "Straight +Y: x unchanged");
    EXPECT_NEAR(ekf.pose.y_cm, -6.0f + expected_dy, 1e-3f, "Straight +Y: y = -6 + pi*4.5");
    EXPECT_NEAR(EKF_GetYawDeg(&ekf), 90.0f, 1e-3f, "Straight +Y: yaw = 90 deg");
    EXPECT_TRUE(is_cov_valid(&ekf), "Straight +Y: covariance valid & symmetric");
}

static void test_inplace_rotation_and_wraparound(void)
{
    EKF_Localizer_t ekf;
    EKF_ResetPose(&ekf, 50.0f, 50.0f, 170.0f, true, 0.0f);

    /* Xoay CCW 360 do qua ranh gioi +180/-180 (180 buoc, moi buoc +2 deg) */
    float true_yaw_deg = 170.0f;
    float imu_deg = 0.0f;
    float arc_per_step = (2.0f * ROBOT_DEG2RAD_F) * (ROBOT_TRACK_WIDTH_CM * 0.5f);
    float tick_accum = 0.0f;
    int32_t prev_ticks = 0;

    for (int i = 0; i < 180; i++) {
        tick_accum += arc_per_step / ROBOT_CM_PER_TICK;
        int32_t cur_ticks = (int32_t)roundf(tick_accum);
        int32_t dticks = cur_ticks - prev_ticks;
        prev_ticks = cur_ticks;

        imu_deg += 2.0f;
        if (imu_deg >= 180.0f) imu_deg -= 360.0f;
        true_yaw_deg += 2.0f;
        if (true_yaw_deg > 180.0f) true_yaw_deg -= 360.0f;

        EKF_Update(&ekf, -dticks, +dticks, 0.02f, true, imu_deg, (2.0f * ROBOT_DEG2RAD_F) / 0.02f);
        EXPECT_TRUE(is_cov_valid(&ekf), "Wraparound step: covariance valid");
    }

    EXPECT_NEAR(ekf.pose.x_cm, 50.0f, 0.05f, "360 spin: x stationary");
    EXPECT_NEAR(ekf.pose.y_cm, 50.0f, 0.05f, "360 spin: y stationary");
    float err_deg = fabsf(EKF_WrapPi(ekf.pose.theta_rad - (170.0f * ROBOT_DEG2RAD_F))) * ROBOT_RAD2DEG_F;
    EXPECT_TRUE(err_deg < 0.5f, "360 spin across +/-180: yaw error < 0.5 deg");
}

static void test_wheel_slip_rejection_with_imu(void)
{
    EKF_Localizer_t ekf;
    EKF_ResetPose(&ekf, 30.0f, 0.0f, 90.0f, true, 0.0f);

    /* Xe xoay that 90 deg CCW trong 1.0s (50 buoc), nhung banh trai bi truot 45% */
    float true_omega = (90.0f * ROBOT_DEG2RAD_F) / 1.0f;
    float imu_deg = 0.0f;
    bool slip_flagged = false;

    for (int i = 0; i < 50; i++) {
        float dt = 0.02f;
        imu_deg += (true_omega * dt) * ROBOT_RAD2DEG_F;
        /* Banh phai chay 10 cm/s, banh trai dang le -10 nhung bi truot chi bao -2 */
        float dr_cm = 0.5f * ROBOT_TRACK_WIDTH_CM * true_omega * dt;
        float dl_cm = -dr_cm * 0.25f; /* Truot nghiem trong! */
        int32_t t_r = (int32_t)roundf(dr_cm / ROBOT_CM_PER_TICK);
        int32_t t_l = (int32_t)roundf(dl_cm / ROBOT_CM_PER_TICK);

        EKF_Update(&ekf, t_l, t_r, dt, true, imu_deg, true_omega);
        if (ekf.wheel_slip_detected) slip_flagged = true;
    }

    float ekf_err_deg  = fabsf(EKF_WrapPi(ekf.pose.theta_rad - (180.0f * ROBOT_DEG2RAD_F))) * ROBOT_RAD2DEG_F;
    float odom_err_deg = fabsf(EKF_WrapPi(ekf.odom_theta_rad - (180.0f * ROBOT_DEG2RAD_F))) * ROBOT_RAD2DEG_F;

    EXPECT_TRUE(slip_flagged, "Wheel slip detector triggers when |omega_enc - omega_gyro| > 0.45");
    EXPECT_TRUE(odom_err_deg > 25.0f, "Pure encoder odom drifts > 25 deg under wheel slip");
    EXPECT_TRUE(ekf_err_deg < 3.0f, "EKF rejects wheel slip via IMU fusion (< 3 deg error)");
}

static void test_frozen_imu_fallback(void)
{
    EKF_Localizer_t ekf;
    EKF_ResetPose(&ekf, 30.0f, 30.0f, 90.0f, true, 0.0f);

    /* IMU bi treo dung yen tai 0 deg trong 1.0s trong khi encoder xoay 90 deg CCW */
    float true_omega = (90.0f * ROBOT_DEG2RAD_F) / 1.0f;
    float arc_step = 0.5f * ROBOT_TRACK_WIDTH_CM * true_omega * 0.02f;
    int32_t ticks = (int32_t)roundf(arc_step / ROBOT_CM_PER_TICK);

    for (int i = 0; i < 50; i++) {
        EKF_Update(&ekf, -ticks, +ticks, 0.02f, true, 0.0f, 0.0f);
    }

    EXPECT_TRUE(ekf.imu_frozen, "Frozen IMU detected during sustained turn");
    /* Sau khi phat hien dong bang (0.24s) va phuc hoi goc encoder, sai so < 3 deg */
    float yaw_deg = EKF_GetYawDeg(&ekf);
    if (yaw_deg < 0.0f) yaw_deg += 360.0f;
    float frozen_err = fabsf(180.0f - yaw_deg);
    EXPECT_TRUE(frozen_err < 3.0f, "EKF recovers heading accurately (< 3 deg) when IMU freezes");
}

static void test_zupt_and_nan_safety(void)
{
    EKF_Localizer_t ekf;
    EKF_ResetPose(&ekf, 30.0f, -6.0f, 90.0f, true, 0.0f);

    /* Dung yen 30 giay (1500 buoc) voi gyro troi nhe 0.02 rad/s (< ZUPT thresh) -> troi tong 15 deg */
    for (int i = 0; i < 1500; i++) {
        EKF_Update(&ekf, 0, 0, 0.02f, true, 0.01f * (float)i, 0.02f);
    }
    EXPECT_TRUE(ekf.zupt_active, "ZUPT activates when ticks=0 and gyro quiet");
    EXPECT_NEAR(ekf.pose.x_cm, 30.0f, 1e-4f, "ZUPT holds x steady");
    EXPECT_NEAR(ekf.pose.y_cm, -6.0f, 1e-4f, "ZUPT holds y steady");
    EXPECT_NEAR(EKF_GetYawDeg(&ekf), 90.0f, 1e-4f, "ZUPT prevents stationary yaw drift");

    /* Chay thang ngay sau 30s ZUPT -> offset IMU phai da neo lai, KHONG duoc nhay 15 deg! */
    for (int i = 0; i < 25; i++) {
        EKF_Update(&ekf, 50, 50, 0.02f, true, 15.0f, 0.0f);
    }
    EXPECT_NEAR(EKF_GetYawDeg(&ekf), 90.0f, 0.5f,
                "No sudden heading jump on first move after 30s ZUPT drift");

    /* Bom NaN / Inf / goc cuc lon 1e9f vao EKF -> khong treo & khong hong trang thai */
    float huge_wrap = EKF_WrapPi(1e9f);
    EXPECT_TRUE(isfinite(huge_wrap) && huge_wrap >= -ROBOT_PI_F - 1e-4f && huge_wrap <= ROBOT_PI_F + 1e-4f,
                "EKF_WrapPi(1e9f) terminates in O(1) within [-pi, pi]");

    EKF_Update(&ekf, 10, 10, NAN, true, NAN, INFINITY);
    EKF_UpdatePosition(&ekf, 31.0f, 1.0f, NAN);
    EKF_UpdateHeading(&ekf, 1.0f, NAN);
    EXPECT_TRUE(isfinite(ekf.pose.x_cm) && isfinite(ekf.pose.y_cm) && isfinite(ekf.pose.theta_rad),
                "NaN/Inf inputs & var=NAN guarded safely");
    EXPECT_TRUE(is_cov_valid(&ekf), "Covariance remains valid after NaN inputs");

    /* Kiem tra chan xung encoder bat thuong (spike clamp) */
    float before_y = ekf.pose.y_cm;
    EKF_Update(&ekf, 100000, 100000, 0.02f, false, 0.0f, 0.0f);
    float jump_cm = fabsf(ekf.pose.y_cm - before_y);
    EXPECT_TRUE(jump_cm < 5.0f, "Corrupted encoder spike clamped to physically plausible displacement");
}

static void test_32bit_encoder_wraparound_delta(void)
{
    /* Mo phong cong thuc tinh delta trong robot_hw_bridge.c qua bien INT32_MAX */
    int32_t prev = INT32_MAX - 5;
    int32_t cur  = (int32_t)((uint32_t)prev + 12U);
    int32_t delta = (int32_t)((uint32_t)cur - (uint32_t)prev);
    EXPECT_TRUE(delta == 12, "Unsigned 32-bit delta handles INT32_MAX wraparound (+12)");

    int32_t prev_neg = INT32_MIN + 3;
    int32_t cur_neg  = (int32_t)((uint32_t)prev_neg - 10U);
    int32_t delta_neg = (int32_t)((uint32_t)cur_neg - (uint32_t)prev_neg);
    EXPECT_TRUE(delta_neg == -10, "Unsigned 32-bit delta handles INT32_MIN underflow (-10)");
}

static void test_position_update(void)
{
    EKF_Localizer_t ekf;
    EKF_Init(&ekf, 30.0f, 0.0f, 90.0f);
    for (int i = 0; i < 20; i++) {
        EKF_Update(&ekf, 80, 80, 0.02f, false, 0.0f, 0.0f);
    }
    float before_x = ekf.pose.x_cm;
    EKF_UpdatePosition(&ekf, 32.0f, ekf.pose.y_cm, 0.5f);
    EXPECT_TRUE(ekf.pose.x_cm > before_x && ekf.pose.x_cm <= 32.0f,
                "EKF_UpdatePosition pulls x toward landmark measurement");
    EXPECT_TRUE(is_cov_valid(&ekf), "Covariance valid after EKF_UpdatePosition");
}

int main(void)
{
    printf("=== Running test_ekf_odometry ===\n");
    test_straight_encoder_odometry();
    test_inplace_rotation_and_wraparound();
    test_wheel_slip_rejection_with_imu();
    test_frozen_imu_fallback();
    test_zupt_and_nan_safety();
    test_32bit_encoder_wraparound_delta();
    test_position_update();

    printf("test_ekf_odometry: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
