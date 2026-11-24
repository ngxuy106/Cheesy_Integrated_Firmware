/**
 * ============================================================================
 * @file    robot_interface.h
 * @brief   HOP DONG GIAO TIEP CHUAN GIUA TANG THAP (BAN BAN) VA TANG CAO (BAN)
 * ============================================================================
 * Nguyen tac tach biet 2 nguoi:
 * - Tang Thap (Core/ + Interface/Src/robot_hw_bridge.c):
 *     Doc Encoder (TIM2/TIM3 qua motor_api), IMU (MPU6050 I2C2), 3x VL53L0X
 *     (I2C1), Pin (ADC1), Nut bam (PA11) -> dong goi vao RobotSensorFrame_t.
 *     Nhan RobotDriveCmd_t -> gioi han gia toc (slew) + bao hoa banh 30cm/s
 *     -> gui toc do goc (rad/s) xuong motor_SetTargetSpeed_Rads().
 * - Tang Cao (HighLevel/):
 *     HOAN TOAN KHONG PHU THUOC STM32 HAL! Chi doc RobotSensorFrame_t va xuat
 *     RobotDriveCmd_t + RobotPose_t + RobotStatus_t. Co the bien dich & mo
 *     phong chay thu truc tiep tren PC bang GCC.
 * ============================================================================
 */

#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Toa do & van toc 2D cua xe trong he toa do sa ban (cm, rad)
 *        Goc (0,0) = goc duoi-trai vung choi; +X sang phai; +Y len tren;
 *        theta = 0 (+X), +pi/2 (+Y huong xuat phat), -pi/2 (-Y ve dich).
 */
typedef struct {
    float x_cm;         /* Toa do X (cm)                        */
    float y_cm;         /* Toa do Y (cm)                        */
    float theta_rad;    /* Goc huong dau xe [-pi, +pi] (rad)    */
    float v_cms;        /* Van toc tinh tien uoc luong (cm/s)   */
    float omega_rads;   /* Van toc goc uoc luong (rad/s, CCW+)  */
} RobotPose_t;

/**
 * @brief Khung du lieu cam bien Tang Thap -> Tang Cao (cap nhat moi 20ms)
 */
typedef struct {
    uint32_t timestamp_ms;      /* Thoi gian he thong (HAL_GetTick)          */
    float    dt_s;              /* Chu ky thuc te giua 2 lan cap nhat (s)    */

    /* Encoder banh trai (g_m0 / TIM2) va phai (g_m1 / TIM3), da chuan hoa chieu */
    int32_t  delta_ticks_left;  /* So xung tang them trong chu ky dt_s       */
    int32_t  delta_ticks_right; /* So xung tang them trong chu ky dt_s       */
    int32_t  total_ticks_left;  /* Tong xung tich luy banh trai              */
    int32_t  total_ticks_right; /* Tong xung tich luy banh phai              */
    float    wheel_left_rads;   /* Toc do goc do duoc banh trai (rad/s)      */
    float    wheel_right_rads;  /* Toc do goc do duoc banh phai (rad/s)      */

    /* IMU MPU6050 (I2C2) */
    bool     imu_valid;         /* true neu IMU da init va doc I2C thanh cong*/
    float    imu_yaw_deg;       /* Goc yaw tich phan 200Hz [-180, 180) (deg) */
    float    imu_gz_rads;       /* Van toc goc truc Z (rad/s, CCW duong)     */

    /* 3 cam bien khoang cach Laser ToF VL53L0X (I2C1) */
    uint16_t tof_left_mm;       /* Cam bien trai (+45 deg), 8190 = ngoai tam */
    uint16_t tof_front_mm;      /* Cam bien truoc (0 deg),  8190 = ngoai tam */
    uint16_t tof_right_mm;      /* Cam bien phai (-45 deg), 8190 = ngoai tam */
    bool     tof_left_valid;
    bool     tof_front_valid;
    bool     tof_right_valid;
    bool     tof_is_stale[3];   /* true = mau cu (chua co phep do moi tu VL53L0X) */

    /* Nguon pin & Nut bam PA11 (Active LOW, Pull-up) */
    float    v_battery;         /* Dien ap pin LiPo 2S (V)                   */
    bool     btn_short_press;   /* Su kien nhan nha (< 1.2s)                 */
    bool     btn_long_press;    /* Su kien nhan giu (>= 1.2s)                */

    /* Lenh dieu khien tu xa tuy chon qua BLE (khong bat buoc khi thi) */
    bool     ble_cmd_start;     /* Lenh bat dau chay tu hanh (RUN:2, alias RUN:3) */
    bool     ble_cmd_estop;
    bool     ble_cmd_reset;
} RobotSensorFrame_t;

/**
 * @brief Lenh dieu khien dong hoc Tang Cao -> Tang Thap
 */
typedef struct {
    float v_linear_cms;         /* Van toc tinh tien mong muon [-30, +30] cm/s */
    float omega_rads;           /* Van toc goc mong muon (rad/s, CCW duong)    */
    bool  brake_stop;           /* true = phanh dung tuc thoi & xoa ramp       */
    bool  robot_is_still;       /* true = cho phep IMU_TrackBias chong troi    */
} RobotDriveCmd_t;

/**
 * @brief Trang thai tom tat Tang Cao -> Tang Thap (cho Telemetry & LED)
 */
typedef struct {
    uint8_t  fsm_state;
    uint8_t  active_segment;
    uint8_t  dynamic_obs_count;
    uint8_t  waypoint_count;
    uint8_t  current_wp_idx;
    bool     replan_done;
    bool     wheel_slip;
    uint32_t elapsed_ms;
    float    active_v_target_cms; /* Van toc v_target dang ap dung o doan hien tai (cm/s) */
    int32_t  estimated_score;
} RobotStatus_t;

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_INTERFACE_H */
