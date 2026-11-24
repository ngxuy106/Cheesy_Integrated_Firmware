/**
 * ============================================================================
 * @file    test_obstacle_mapper.c
 * @brief   Unit tests cho bo quet map & phat hien vat can dong 3x VL53L0X
 * ============================================================================
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "robot_config.h"
#include "arena_map.h"
#include "obstacle_mapper.h"

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_TRUE(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("[FAIL] Line %d: %s\n", __LINE__, msg); } \
} while (0)

static RobotSensorFrame_t make_tof(uint16_t l_mm, uint16_t f_mm, uint16_t r_mm)
{
    RobotSensorFrame_t s;
    memset(&s, 0, sizeof(s));
    s.tof_left_mm     = l_mm;
    s.tof_front_mm    = f_mm;
    s.tof_right_mm    = r_mm;
    s.tof_left_valid  = (l_mm < TOF_OUT_OF_RANGE_MM);
    s.tof_front_valid = (f_mm < TOF_OUT_OF_RANGE_MM);
    s.tof_right_valid = (r_mm < TOF_OUT_OF_RANGE_MM);
    return s;
}

static void test_static_pillars_and_walls_rejected(void)
{
    ObstacleMapper_t mapper;
    ObstacleMapper_Init(&mapper);

    /* Xe o (130, 100) huong +X (0 deg), nhin thang vao tuong phai x=180 (cách 45cm)
     * va tia trai +45 nhin tuong tren y=120 -> khong duoc bao vat can dong! */
    RobotPose_t pose = { 150.0f, 100.0f, 0.0f, 0.0f, 0.0f };
    /* Tuong phai x=180 cach tam xe 30cm -> mat ToF truoc (offset 5cm) do 250mm */
    /* Tuong tren y=120 theo goc +45 -> cach tam xe 20*sqrt(2)=28.3cm -> offset 4.5 -> 238mm */
    RobotSensorFrame_t s = make_tof(238U, 250U, 8190U);

    for (int i = 0; i < 5; i++) {
        ObstacleScanResult_t r = ObstacleMapper_Update(&mapper, &pose, &s, true, false);
        EXPECT_TRUE(!r.new_obstacle_found, "Static walls must not trigger dynamic obstacle");
    }

    /* Xe nhin thang vao coc tinh T14 tai (125, 85), mat trai T14 tai x=120 */
    RobotPose_t pose2 = { 100.0f, 85.0f, 0.0f, 0.0f, 0.0f };
    /* Mat T14 cach tam xe 20cm -> ToF front (offset 5cm) do 150mm */
    RobotSensorFrame_t s2 = make_tof(8190U, 150U, 8190U);
    for (int i = 0; i < 5; i++) {
        ObstacleScanResult_t r = ObstacleMapper_Update(&mapper, &pose2, &s2, true, false);
        EXPECT_TRUE(!r.new_obstacle_found, "Static pillar T14 must be ignored");
    }
    EXPECT_TRUE(ObstacleMapper_GetCount(&mapper) == 0U, "Zero false positives from static scene");
}

static void test_three_frame_debounce_and_side_ray_detection(void)
{
    ObstacleMapper_t mapper;
    ObstacleMapper_Init(&mapper);

    /* Vat can ngau nhien MODE_B tai (125, 65), xe o (145, 85) huong Nam (-90 deg)
     * Goc toi (125,65) tu (145,85) la -135 deg = theta (-90) - 45 deg = tia PHAI (Right -45)!
     * Khoang cach tam-tam = sqrt(20^2 + 20^2) = 28.28 cm
     * Mat coc cach tam xe ~ 28.28 - 5.0 = 23.28 cm -> ToF Right (offset 4.5) do ~ 188 mm */
    RobotPose_t pose = { 145.0f, 85.0f, -ROBOT_PI_F * 0.5f, 0.0f, 0.0f };
    RobotSensorFrame_t s = make_tof(8190U, 8190U, 188U);

    ObstacleScanResult_t r1 = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
    ObstacleScanResult_t r2 = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
    EXPECT_TRUE(!r1.new_obstacle_found && !r2.new_obstacle_found,
                "1-2 frames must NOT confirm obstacle (debounce = 3)");

    ObstacleScanResult_t r3 = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
    EXPECT_TRUE(r3.new_obstacle_found, "3rd clustered frame on Right ToF confirms MODE_B");
    EXPECT_TRUE(fabsf(r3.new_obs_x_cm - 125.0f) < 2.5f && fabsf(r3.new_obs_y_cm - 65.0f) < 2.5f,
                "Estimated obstacle center matches (125, 65) within 2.5cm");

    /* Khung thu 4..6 khong duoc dang ky trung lap */
    for (int i = 0; i < 4; i++) {
        ObstacleScanResult_t rdup = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
        EXPECT_TRUE(!rdup.new_obstacle_found, "Duplicate detections deduplicated");
    }
    EXPECT_TRUE(ObstacleMapper_GetCount(&mapper) == 1U, "Exactly 1 dynamic obstacle recorded");
}

static void test_transient_spike_and_emergency_guard(void)
{
    ObstacleMapper_t mapper;
    ObstacleMapper_Init(&mapper);

    RobotPose_t pose = { 130.0f, 60.0f, -ROBOT_PI_F * 0.5f, 0.0f, 0.0f };
    /* 2 khung nhieu roi mat -> khong confirm */
    RobotSensorFrame_t s_hit = make_tof(8190U, 200U, 8190U);
    RobotSensorFrame_t s_clr = make_tof(8190U, 8190U, 8190U);

    (void)ObstacleMapper_Update(&mapper, &pose, &s_hit, true, true);
    (void)ObstacleMapper_Update(&mapper, &pose, &s_hit, true, true);
    (void)ObstacleMapper_Update(&mapper, &pose, &s_clr, true, true);
    ObstacleScanResult_t r4 = ObstacleMapper_Update(&mapper, &pose, &s_hit, true, true);
    EXPECT_TRUE(!r4.new_obstacle_found, "Transient spike resets debounce streak");

    /* Co bao khan cap phia truoc < 85mm phai bat ngay o frame 1 */
    RobotSensorFrame_t s_emg = make_tof(8190U, 70U, 8190U);
    ObstacleScanResult_t remg = ObstacleMapper_Update(&mapper, &pose, &s_emg, true, true);
    EXPECT_TRUE(remg.front_emergency, "Emergency front guard triggers immediately at 70mm");
}

static void test_stale_tof_and_gate_adjacent_obstacles(void)
{
    ObstacleMapper_t mapper;
    ObstacleMapper_Init(&mapper);

    /* 1. 1 mau nhieu + 2 ban sao cu (tof_is_stale=true) KHONG duoc confirm vat can gia */
    RobotPose_t pose = { 130.0f, 80.0f, -ROBOT_PI_F * 0.5f, 0.0f, 0.0f };
    RobotSensorFrame_t s = make_tof(8190U, 200U, 8190U);
    s.tof_is_stale[1] = false;
    ObstacleScanResult_t r1 = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
    s.tof_is_stale[1] = true;
    ObstacleScanResult_t r2 = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
    ObstacleScanResult_t r3 = ObstacleMapper_Update(&mapper, &pose, &s, true, true);
    EXPECT_TRUE(!r1.new_obstacle_found && !r2.new_obstacle_found && !r3.new_obstacle_found,
                "Stale ToF samples must NOT falsely advance debounce streak");
    EXPECT_TRUE(ObstacleMapper_GetCount(&mapper) == 0U, "Zero obstacles from 1 fresh + 2 stale frames");

    /* 2. Phat hien vat can MODE_E (145, 5) sat vach dich & ngay duoi T16 bang tia Trai +45 */
    ObstacleMapper_Init(&mapper);
    RobotPose_t pose_gate = { 131.0f, 15.0f, -ROBOT_PI_F * 0.5f, 0.0f, 0.0f };
    /* Tia trai +45 (-45 deg world) tu (131,15) cham mat trai MODE_E x=140 tai (140, 6):
     * d_hit = 9*sqrt(2) = 12.73cm -> d_meas = 12.73 - 4.5 = 8.23cm = 82mm */
    RobotSensorFrame_t s_gate = make_tof(82U, 8190U, 8190U);
    bool found_e = false;
    for (int i = 0; i < 3; i++) {
        ObstacleScanResult_t rg = ObstacleMapper_Update(&mapper, &pose_gate, &s_gate, true, true);
        if (rg.new_obstacle_found) found_e = true;
    }
    EXPECT_TRUE(found_e && ObstacleMapper_GetCount(&mapper) == 1U,
                "MODE_E (145, 5) right beneath T16 & finish line detected cleanly");
}

static void test_o3_scan_zone_static_reject_and_dynamic_detect(void)
{
    ObstacleMapper_t mapper;
    ObstacleMapper_Init(&mapper);

    /* 1. Cac coc tinh trong O 3 (T9, T10, T11, T12) khong duoc bao dong gia khi bat quet O 3..O 5 */
    RobotPose_t pose_t11 = { 65.0f, 100.0f, 0.0f, 0.0f, 0.0f }; /* Huong +X nhin T11 (85, 100) */
    RobotSensorFrame_t s_t11 = make_tof(8190U, 100U, 8190U);    /* Mat T11 o x=80 -> d_meas=10cm */
    RobotPose_t pose_t12 = { 95.0f, 85.0f, -ROBOT_PI_F * 0.5f, 0.0f, 0.0f }; /* Nhin T12 (95,65) */
    RobotSensorFrame_t s_t12 = make_tof(8190U, 100U, 8190U);    /* Mat T12 o y=70 -> d_meas=10cm */
    for (int i = 0; i < 4; i++) {
        ObstacleScanResult_t r1 = ObstacleMapper_Update(&mapper, &pose_t11, &s_t11, true, true);
        ObstacleScanResult_t r2 = ObstacleMapper_Update(&mapper, &pose_t12, &s_t12, true, true);
        EXPECT_TRUE(!r1.new_obstacle_found && !r2.new_obstacle_found,
                    "Static O3 pillars T11/T12 must NOT trigger dynamic obstacle");
    }
    EXPECT_TRUE(ObstacleMapper_GetCount(&mapper) == 0U, "Zero false positives on O3 static pillars");

    /* 2. Vat can trong O 2 (x < 60cm) bi bo qua khi restrict_to_o3_o5 == true */
    RobotPose_t pose_o2 = { 45.0f, 65.0f, ROBOT_PI_F * 0.5f, 0.0f, 0.0f };
    RobotSensorFrame_t s_o2 = make_tof(8190U, 100U, 8190U); /* (45, 85) -> x < 60 */
    for (int i = 0; i < 4; i++) {
        ObstacleScanResult_t ro2 = ObstacleMapper_Update(&mapper, &pose_o2, &s_o2, true, true);
        EXPECT_TRUE(!ro2.new_obstacle_found, "O2 zone (x < 60cm) ignored when restricted to O3..O5");
    }

    /* 3. Vat can dong dat trong O 3 tai (102, 95) phai duoc xac nhan sau 3 mau tuoi */
    RobotPose_t pose_o3 = { 102.0f, 75.0f, ROBOT_PI_F * 0.5f, 0.0f, 0.0f };
    RobotSensorFrame_t s_o3 = make_tof(8190U, 100U, 8190U); /* Mat (102, 90) -> d_meas=10cm */
    bool found_o3 = false;
    for (int i = 0; i < 3; i++) {
        ObstacleScanResult_t ro3 = ObstacleMapper_Update(&mapper, &pose_o3, &s_o3, true, true);
        if (ro3.new_obstacle_found) found_o3 = true;
    }
    EXPECT_TRUE(found_o3 && ObstacleMapper_GetCount(&mapper) == 1U,
                "Dynamic obstacle in O3 (102, 95) detected when restrict_to_o3_o5 == true");
}

int main(void)
{
    printf("=== Running test_obstacle_mapper ===\n");
    test_static_pillars_and_walls_rejected();
    test_three_frame_debounce_and_side_ray_detection();
    test_transient_spike_and_emergency_guard();
    test_stale_tof_and_gate_adjacent_obstacles();
    test_o3_scan_zone_static_reject_and_dynamic_detect();

    printf("test_obstacle_mapper: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
