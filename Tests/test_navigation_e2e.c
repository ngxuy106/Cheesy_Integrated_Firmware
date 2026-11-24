/**
 * ============================================================================
 * @file    test_navigation_e2e.c
 * @brief   Mo phong vong kin 2D full sa ban (Closed-Loop Physics + 3-Ray ToF
 *          Raycaster + Noisy Encoder/IMU + Unified Single-Run FSM + Tunable v_target)
 * ============================================================================
 * Kiem thu toan dien:
 * 1. Gop chung 1 che do chay duy nhat: 2 lan chay lien tiep bat dau bang nut PA11,
 *    deu tu bat 3x ToF quet map ngay khi vao O 3 (x >= 60cm) va co toc do nhu nhau.
 * 2. Hieu chinh v_target linh hoat (dong nhat hoac tung doan O 1..O 5) va giu
 *    nguyen sau Mission_ResetAll(), kep an toan [10, 30] cm/s.
 * 3. 11 vi tri vat can ngau nhien (O 3 + MODE_A..E + cua dich O 5): xe TU QUET
 *    3x ToF, TU REPLAN A*, KHONG VA CHAM (Corner >= 0.50cm, Rec=0)!
 * ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "robot_config.h"
#include "robot_interface.h"
#include "arena_map.h"
#include "astar_planner.h"
#include "pure_pursuit.h"
#include "mission_fsm.h"

static int g_pass = 0;
static int g_fail = 0;

#define EXPECT_TRUE(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; printf("[FAIL] Line %d: %s\n", __LINE__, msg); } \
} while (0)

typedef struct {
    float x, y, theta;      /* Ground-truth pose (cm, rad) */
    float cur_v, cur_w;     /* Slew-limited actual velocities */
    float tick_rem_l, tick_rem_r;
    int32_t total_l, total_r;
    float imu_yaw_deg;
    bool  has_dyn_obs;
    float dyn_x, dyn_y;     /* Ground-truth 10x10cm dynamic obstacle center */
} SimWorld_t;

static float slew_sim(float cur, float target, float step)
{
    float d = target - cur;
    if (d >  step) return cur + step;
    if (d < -step) return cur - step;
    return target;
}

static float dist_box(float px, float py, float x0, float y0, float x1, float y1)
{
    float dx = (px < x0) ? (x0 - px) : ((px > x1) ? (px - x1) : 0.0f);
    float dy = (py < y0) ? (y0 - py) : ((py > y1) ? (py - y1) : 0.0f);
    return sqrtf(dx * dx + dy * dy);
}

static float min_true_clearance_cm(const SimWorld_t *w, float x, float y)
{
    float d = Map_DistToStaticObstacleCm(x, y);
    if (w->has_dyn_obs) {
        float dd = dist_box(x, y, w->dyn_x - 5.0f, w->dyn_y - 5.0f,
                                  w->dyn_x + 5.0f, w->dyn_y + 5.0f);
        if (dd < d) d = dd;
    }
    return d;
}

/* Kiem tra khe ho thuc cua khung xe hinh chu nhat 11.46 x 10.0 cm (4 goc + 4 trung diem canh) */
static float min_chassis_corner_clearance_cm(const SimWorld_t *w)
{
    const float hl = 5.0f;
    const float hw = 5.73f;
    const float c  = cosf(w->theta);
    const float s  = sinf(w->theta);
    const float pts[8][2] = {
        { hl, hw}, { hl,-hw}, {-hl, hw}, {-hl,-hw},
        { hl, 0 }, {-hl, 0 }, { 0,  hw}, { 0, -hw}
    };
    float best = 999.0f;
    for (int i = 0; i < 8; i++) {
        float wx = w->x + pts[i][0] * c - pts[i][1] * s;
        float wy = w->y + pts[i][0] * s + pts[i][1] * c;
        if (wy < 0.0f) continue;
        float d = min_true_clearance_cm(w, wx, wy);
        if (d < best) best = d;
    }
    return best;
}

/* Raycast that cua 1 tia VL53L0X trong sa ban (do phan giai 0.5cm) */
static uint16_t raycast_tof_mm(const SimWorld_t *w, float angle_offset_deg, float mount_offset_cm)
{
    float phi = w->theta + angle_offset_deg * ROBOT_DEG2RAD_F;
    float c = cosf(phi), s = sinf(phi);
    float ox = w->x + mount_offset_cm * c;
    float oy = w->y + mount_offset_cm * s;

    for (float d = 0.5f; d <= 80.0f; d += 0.5f) {
        float px = ox + d * c;
        float py = oy + d * s;
        if (px <= 0.0f || px >= 180.0f || py >= 120.0f) {
            return (uint16_t)roundf(d * 10.0f);
        }
        /* Khoi chu U */
        if (px >= 60.0f && px <= 120.0f && py >= 0.0f && py <= 60.0f) {
            return (uint16_t)roundf(d * 10.0f);
        }
        /* 17 coc tinh */
        for (int i = 0; i < NUM_STD_OBSTACLES; i++) {
            float cx = (float)OBSTACLES_STD[i].center_x_mm * 0.1f;
            float cy = (float)OBSTACLES_STD[i].center_y_mm * 0.1f;
            if (px >= cx - 5.0f && px <= cx + 5.0f && py >= cy - 5.0f && py <= cy + 5.0f) {
                return (uint16_t)roundf(d * 10.0f);
            }
        }
        /* Coc dong ngau nhien */
        if (w->has_dyn_obs) {
            if (px >= w->dyn_x - 5.0f && px <= w->dyn_x + 5.0f &&
                py >= w->dyn_y - 5.0f && py <= w->dyn_y + 5.0f) {
                return (uint16_t)roundf(d * 10.0f);
            }
        }
    }
    return TOF_OUT_OF_RANGE_MM;
}

static void sim_init_world(SimWorld_t *w, bool has_dyn, float dyn_x, float dyn_y)
{
    memset(w, 0, sizeof(*w));
    w->x           = ROBOT_START_X;
    w->y           = ROBOT_START_Y;
    w->theta       = ROBOT_START_YAW * ROBOT_DEG2RAD_F;
    w->imu_yaw_deg = 0.0f;
    w->has_dyn_obs = has_dyn;
    w->dyn_x       = dyn_x;
    w->dyn_y       = dyn_y;
}

static void sim_step_physics(SimWorld_t *w, const RobotDriveCmd_t *cmd, float dt,
                             RobotSensorFrame_t *out_frame, float *io_max_wheel_cms)
{
    if (cmd->brake_stop) {
        w->cur_v = 0.0f;
        w->cur_w = 0.0f;
    } else {
        float tv = fmaxf(-ROBOT_MAX_SPEED_CMS, fminf(ROBOT_MAX_SPEED_CMS, cmd->v_linear_cms));
        float tw = fmaxf(-ROBOT_MAX_OMEGA_RADS, fminf(ROBOT_MAX_OMEGA_RADS, cmd->omega_rads));
        w->cur_v = slew_sim(w->cur_v, tv, ROBOT_ACCEL_MAX_CMS2 * dt);
        w->cur_w = slew_sim(w->cur_w, tw, ROBOT_ALPHA_MAX_RADS2 * dt);
    }

    float half_wb = ROBOT_TRACK_WIDTH_CM * 0.5f;
    float vl = w->cur_v - w->cur_w * half_wb;
    float vr = w->cur_v + w->cur_w * half_wb;
    float peak = fmaxf(fabsf(vl), fabsf(vr));
    if (peak > ROBOT_MAX_WHEEL_SPEED_CMS) {
        float sc = ROBOT_MAX_WHEEL_SPEED_CMS / peak;
        vl *= sc;
        vr *= sc;
        w->cur_v = 0.5f * (vl + vr);
        w->cur_w = (vr - vl) / ROBOT_TRACK_WIDTH_CM;
    }
    if (peak > *io_max_wheel_cms) {
        *io_max_wheel_cms = fminf(peak, ROBOT_MAX_WHEEL_SPEED_CMS);
    }

    /* Tich phan dong hoc thuc te */
    float dtheta = w->cur_w * dt;
    float ds     = w->cur_v * dt;
    w->x += ds * cosf(w->theta + 0.5f * dtheta);
    w->y += ds * sinf(w->theta + 0.5f * dtheta);
    w->theta = EKF_WrapPi(w->theta + dtheta);

    /* Xung encoder luong tu hoa 2800 PPR (+1% chenh lech banh thuc te de test EKF) */
    float dl_ticks_f = (vl * 1.005f * dt) / ROBOT_CM_PER_TICK + w->tick_rem_l;
    float dr_ticks_f = (vr * 0.995f * dt) / ROBOT_CM_PER_TICK + w->tick_rem_r;
    int32_t dl_ticks = (int32_t)floorf(dl_ticks_f + 0.5f);
    int32_t dr_ticks = (int32_t)floorf(dr_ticks_f + 0.5f);
    w->tick_rem_l = dl_ticks_f - (float)dl_ticks;
    w->tick_rem_r = dr_ticks_f - (float)dr_ticks;
    w->total_l += dl_ticks;
    w->total_r += dr_ticks;

    /* IMU yaw tich phan 200Hz */
    w->imu_yaw_deg += dtheta * ROBOT_RAD2DEG_F;
    while (w->imu_yaw_deg >= 180.0f) w->imu_yaw_deg -= 360.0f;
    while (w->imu_yaw_deg < -180.0f) w->imu_yaw_deg += 360.0f;

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->dt_s              = dt;
    out_frame->delta_ticks_left  = dl_ticks;
    out_frame->delta_ticks_right = dr_ticks;
    out_frame->total_ticks_left  = w->total_l;
    out_frame->total_ticks_right = w->total_r;
    out_frame->wheel_left_rads   = vl / ROBOT_WHEEL_RADIUS_CM;
    out_frame->wheel_right_rads  = vr / ROBOT_WHEEL_RADIUS_CM;
    out_frame->imu_valid         = true;
    out_frame->imu_yaw_deg       = w->imu_yaw_deg;
    out_frame->imu_gz_rads       = w->cur_w;
    out_frame->tof_left_mm       = raycast_tof_mm(w, TOF_ANGLE_LEFT_DEG,  TOF_OFFSET_LEFT_CM);
    out_frame->tof_front_mm      = raycast_tof_mm(w, TOF_ANGLE_FRONT_DEG, TOF_OFFSET_FRONT_CM);
    out_frame->tof_right_mm      = raycast_tof_mm(w, TOF_ANGLE_RIGHT_DEG, TOF_OFFSET_RIGHT_CM);
    out_frame->tof_left_valid    = (out_frame->tof_left_mm  < TOF_OUT_OF_RANGE_MM);
    out_frame->tof_front_valid   = (out_frame->tof_front_mm < TOF_OUT_OF_RANGE_MM);
    out_frame->tof_right_valid   = (out_frame->tof_right_mm < TOF_OUT_OF_RANGE_MM);
    out_frame->v_battery         = 7.95f;
}

static void run_single_clean_pass(float *out_max_v_cms, float *out_max_o3_v_cms,
                                  float *out_max_wheel, float *out_min_clr,
                                  float *out_ekf_err, bool *out_saw_o3_scan,
                                  uint8_t *out_dyn_obs, float *out_v_targets)
{
    SimWorld_t world;
    sim_init_world(&world, false, 0.0f, 0.0f);

    RobotSensorFrame_t frame;
    RobotDriveCmd_t cmd = {0};
    memset(&frame, 0, sizeof(frame));
    frame.dt_s = 0.02f;
    frame.imu_valid = true;
    frame.v_battery = 8.0f;
    frame.btn_short_press = true;
    Mission_Step(&frame, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_RUN_O1_O2,
                "Unified run starts in FSM_RUN_O1_O2 on PA11 short press");

    float max_v_cms = 0.0f;
    float max_o3_v  = 0.0f;
    float max_wheel = 0.0f;
    float min_clr = 999.0f;
    bool saw_o3_scan = false;

    for (int step = 0; step < 2500; step++) {
        sim_step_physics(&world, &cmd, 0.02f, &frame, &max_wheel);
        float abs_v = fabsf(world.cur_v);
        if (abs_v > max_v_cms) max_v_cms = abs_v;
        if (world.x >= 60.0f && world.x < 120.0f && abs_v > max_o3_v) {
            max_o3_v = abs_v;
        }
        Mission_Step(&frame, &cmd);
        if (Mission_GetState() == FSM_RUN_SCAN_O3_O5) {
            saw_o3_scan = true;
        }
        if (world.y >= 0.0f) {
            float clr = min_true_clearance_cm(&world, world.x, world.y);
            if (clr < min_clr) min_clr = clr;
        }
        if (Mission_GetState() == FSM_MISSION_FINISHED ||
            Mission_GetState() == FSM_EMERGENCY_STOP) {
            break;
        }
    }

    RobotPose_t est = Mission_GetPose();
    *out_max_v_cms    = max_v_cms;
    *out_max_o3_v_cms = max_o3_v;
    *out_max_wheel    = max_wheel;
    *out_min_clr      = min_clr;
    *out_ekf_err      = hypotf(est.x_cm - world.x, est.y_cm - world.y);
    *out_saw_o3_scan  = saw_o3_scan;
    *out_dyn_obs      = ObstacleMapper_GetCount(Mission_GetMapper());
    for (int s = 0; s < TRACK_SEG_COUNT; s++) {
        out_v_targets[s] = Mission_GetSegmentTargetSpeed((TrackSegment_t)s);
    }
}

static void test_unified_consecutive_runs_identical_speed(void)
{
    Mission_Init();

    float max_v1 = 0.0f, o3_v1 = 0.0f, max_w1 = 0.0f, min_c1 = 0.0f, err1 = 0.0f, vt1[TRACK_SEG_COUNT];
    bool scan1 = false;
    uint8_t dyn1 = 0U;
    run_single_clean_pass(&max_v1, &o3_v1, &max_w1, &min_c1, &err1, &scan1, &dyn1, vt1);
    EXPECT_TRUE(Mission_GetState() == FSM_MISSION_FINISHED, "Run 1 reaches FSM_MISSION_FINISHED");
    EXPECT_TRUE(scan1, "Run 1 enables ToF scanning in O 3 (FSM_RUN_SCAN_O3_O5)");
    EXPECT_TRUE(dyn1 == 0U, "Run 1 has zero false dynamic obstacles on O3..O5 static pillars");
    EXPECT_TRUE(min_c1 >= 5.5f, "Run 1 maintains clearance >= 5.5cm");
    EXPECT_TRUE(err1 < 3.0f, "Run 1 EKF error < 3.0cm");
    EXPECT_TRUE(max_w1 <= ROBOT_MAX_WHEEL_SPEED_CMS + 0.05f, "Run 1 wheel <= 30 cm/s");

    /* Bam nut PA11 reset ve IDLE roi chay tiep lan 2: toc do & logic PHAI giong het lan 1 */
    RobotSensorFrame_t rst;
    RobotDriveCmd_t cmd;
    memset(&rst, 0, sizeof(rst));
    rst.dt_s = 0.02f;
    rst.imu_valid = true;
    rst.v_battery = 8.0f;
    rst.btn_short_press = true;
    Mission_Step(&rst, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_IDLE, "PA11 resets finished run back to FSM_IDLE");

    float max_v2 = 0.0f, o3_v2 = 0.0f, max_w2 = 0.0f, min_c2 = 0.0f, err2 = 0.0f, vt2[TRACK_SEG_COUNT];
    bool scan2 = false;
    uint8_t dyn2 = 0U;
    run_single_clean_pass(&max_v2, &o3_v2, &max_w2, &min_c2, &err2, &scan2, &dyn2, vt2);
    EXPECT_TRUE(Mission_GetState() == FSM_MISSION_FINISHED, "Run 2 reaches FSM_MISSION_FINISHED");
    EXPECT_TRUE(scan2, "Run 2 also enables ToF scanning in O 3 (FSM_RUN_SCAN_O3_O5)");
    EXPECT_TRUE(dyn2 == 0U, "Run 2 also has zero false dynamic obstacles");
    EXPECT_TRUE(min_c2 >= 5.5f, "Run 2 maintains clearance >= 5.5cm");
    EXPECT_TRUE(err2 < 3.0f, "Run 2 EKF error < 3.0cm");
    EXPECT_TRUE(fabsf(max_v1 - max_v2) < 0.2f && fabsf(max_w1 - max_w2) < 0.5f,
                "Consecutive runs execute at identical speed");
    for (int s = 0; s < TRACK_SEG_COUNT; s++) {
        EXPECT_TRUE(fabsf(vt1[s] - vt2[s]) < 1e-4f, "v_target identical between Run 1 and Run 2");
    }

    printf("  [Unified Run 1 & 2] Run1 MaxV=%.1f MaxW=%.1f Err=%.2fcm | Run2 MaxV=%.1f MaxW=%.1f Err=%.2fcm | ScanO3=(%d,%d) DynObs=(%u,%u)\n",
           max_v1, max_w1, err1, max_v2, max_w2, err2, (int)scan1, (int)scan2, (unsigned)dyn1, (unsigned)dyn2);
}

static void test_tunable_v_target_persists_across_reset(void)
{
    Mission_Init();

    /* 1. Kiem tra kep nguong & tu choi NaN */
    EXPECT_TRUE(!Mission_SetUniformTargetSpeed(NAN), "Reject NaN v_target");
    EXPECT_TRUE(Mission_SetSegmentTargetSpeed(TRACK_SEG_O1_START_NARROW, 5.0f), "Accept low input -> clamp");
    EXPECT_TRUE(fabsf(Mission_GetSegmentTargetSpeed(TRACK_SEG_O1_START_NARROW) - ROBOT_MIN_CRUISE_SPEED_CMS) < 1e-4f,
                "Low v_target clamped to 10.0 cm/s");
    EXPECT_TRUE(Mission_SetSegmentTargetSpeed(TRACK_SEG_O3_STRAIGHT, 50.0f), "Accept high input -> clamp");
    EXPECT_TRUE(fabsf(Mission_GetSegmentTargetSpeed(TRACK_SEG_O3_STRAIGHT) - ROBOT_MAX_SPEED_CMS) < 1e-4f,
                "High v_target clamped to 30.0 cm/s");

    /* 2. Dat dong nhat 16 cm/s -> Reset -> phai giu nguyen 16 cm/s va chay v_linear <= 16.05 cm/s */
    EXPECT_TRUE(Mission_SetUniformTargetSpeed(16.0f), "Set uniform v_target = 16.0 cm/s");
    Mission_ResetAll(true, 0.0f);
    for (int s = 0; s < TRACK_SEG_COUNT; s++) {
        EXPECT_TRUE(fabsf(Mission_GetSegmentTargetSpeed((TrackSegment_t)s) - 16.0f) < 1e-4f,
                    "Tuned v_target persists across Mission_ResetAll");
    }

    float max_v16 = 0.0f, o3_v16 = 0.0f, max_w16 = 0.0f, min_c = 0.0f, err = 0.0f, vt[TRACK_SEG_COUNT];
    bool scan = false;
    uint8_t dyn = 0U;
    run_single_clean_pass(&max_v16, &o3_v16, &max_w16, &min_c, &err, &scan, &dyn, vt);
    EXPECT_TRUE(Mission_GetState() == FSM_MISSION_FINISHED, "Uniform 16 cm/s run finishes");
    EXPECT_TRUE(max_v16 <= 16.05f && max_w16 <= ROBOT_MAX_WHEEL_SPEED_CMS + 0.05f,
                "Uniform 16 cm/s run respects max linear speed <= 16.05 cm/s");

    /* 3. Hieu chinh rieng O 3 len 30.0 cm/s va chay lai -> O 3 dat >= 28.0 cm/s & banh <= 30.05 cm/s */
    Mission_ResetAll(true, 0.0f);
    EXPECT_TRUE(Mission_SetSegmentTargetSpeed(TRACK_SEG_O3_STRAIGHT, 30.0f), "Tune O3 speed to 30 cm/s");
    float max_v30 = 0.0f, o3_v30 = 0.0f, max_w30 = 0.0f;
    run_single_clean_pass(&max_v30, &o3_v30, &max_w30, &min_c, &err, &scan, &dyn, vt);
    EXPECT_TRUE(Mission_GetState() == FSM_MISSION_FINISHED, "Tuned O3=30 cm/s run finishes");
    EXPECT_TRUE(o3_v30 >= 28.0f && max_v30 <= ROBOT_MAX_SPEED_CMS + 0.05f &&
                max_w30 <= ROBOT_MAX_WHEEL_SPEED_CMS + 0.05f,
                "Tuned O3=30 cm/s achieves >= 28 cm/s in O3 while staying <= 30 cm/s");

    printf("  [Tunable v_target] Uniform16 -> MaxV=%.1f (MaxW=%.1f) | O3=30 -> O3_V=%.1f (MaxW=%.1f)\n",
           max_v16, max_w16, o3_v30, max_w30);
}

static void run_random_obstacle_scenario(const char *label, float obs_x, float obs_y)
{
    SimWorld_t world;
    sim_init_world(&world, true, obs_x, obs_y);
    Mission_Init();

    RobotSensorFrame_t frame;
    RobotDriveCmd_t cmd = {0};
    memset(&frame, 0, sizeof(frame));
    frame.dt_s = 0.02f;
    frame.imu_valid = true;
    frame.v_battery = 8.0f;
    frame.btn_short_press = true; /* Nhan nut PA11 binh thuong - chi 1 che do chay duy nhat */
    Mission_Step(&frame, &cmd);

    float max_wheel = 0.0f;
    float min_clr = 999.0f;
    float min_corner = 999.0f;
    int rec_ticks = 0;
    bool saw_o3_scan = false;

    for (int step = 0; step < 2500; step++) {
        sim_step_physics(&world, &cmd, 0.02f, &frame, &max_wheel);
        Mission_Step(&frame, &cmd);
        if (Mission_GetState() == FSM_RUN_SCAN_O3_O5) saw_o3_scan = true;
        if (Mission_GetState() == FSM_RECOVERY_BACKUP) rec_ticks++;
        if (world.y >= 0.0f) {
            float clr = min_true_clearance_cm(&world, world.x, world.y);
            if (clr < min_clr) min_clr = clr;
            float cc = min_chassis_corner_clearance_cm(&world);
            if (cc < min_corner) min_corner = cc;
        }
        if (Mission_GetState() == FSM_MISSION_FINISHED ||
            Mission_GetState() == FSM_EMERGENCY_STOP) {
            break;
        }
    }

    RobotStatus_t st = Mission_GetStatus();
    printf("  [%s obs=(%.0f,%.0f)] State=%d, ScanO3=%d, True=(%.1f,%.1f), DynObs=%u, Replan=%d, MinClr=%.2fcm, Corner=%.2fcm, Rec=%d, Score=%d\n",
           label, obs_x, obs_y, (int)st.fsm_state, (int)saw_o3_scan, world.x, world.y,
           (unsigned)st.dynamic_obs_count, (int)st.replan_done, min_clr, min_corner, rec_ticks, (int)st.estimated_score);

    EXPECT_TRUE(saw_o3_scan, "Entered FSM_RUN_SCAN_O3_O5 in O 3");
    EXPECT_TRUE(st.fsm_state == FSM_MISSION_FINISHED, label);
    EXPECT_TRUE(st.dynamic_obs_count >= 1U && st.replan_done, "Dynamic obstacle detected & replanned");
    EXPECT_TRUE(min_clr >= 5.75f, "Random obstacle scenario: center clearance >= 5.75cm");
    EXPECT_TRUE(min_corner >= 0.50f, "Random obstacle scenario: 11.46x10cm chassis corner clearance >= 0.50cm");
    EXPECT_TRUE(rec_ticks == 0, "Random obstacle scenario: zero emergency recovery backups");
    EXPECT_TRUE(max_wheel <= ROBOT_MAX_WHEEL_SPEED_CMS + 0.05f, "Random obstacle scenario: wheel <= 30cm/s");
}

static void test_map_geometry_and_pp_curvature(void)
{
    int violations = 0;
    for (int r = 1; r < MAP_NY; r++) {
        for (int c = 0; c < MAP_NX; c++) {
            if (Map_GetCellType(r, c) == CELL_FREE) {
                float d = Map_DistToStaticObstacleCm(Map_GetWorldX(c), Map_GetWorldY(r));
                if (d < 6.768f) violations++;
            }
        }
    }
    EXPECT_TRUE(violations == 0, "All CELL_FREE cells satisfy field_map_final R=5.0cm clearance >= 6.768cm");

    PurePursuit_Init(7.5f, 30.0f);
    Waypoint_t wps[2] = { {0.0f, 0.0f}, {0.0f, 30.0f} };
    RobotPose_t p = { 4.2f, 0.0f, 70.0f * ROBOT_DEG2RAD_F, 30.0f, 0.0f };
    PurePursuit_Cmd_t pcmd;
    bool ok = PurePursuit_Compute(p, wps, 2, &pcmd);
    PurePursuit_t pst = PurePursuit_GetStatus();
    EXPECT_TRUE(ok && !pcmd.is_pivoting, "PurePursuit computes curved arc below pivot threshold");
    float act_k = (pcmd.v_linear_cms > 0.1f) ? (pcmd.omega_rads / pcmd.v_linear_cms) : 0.0f;
    EXPECT_TRUE(fabsf(act_k - pst.current_curvature) < 1e-3f,
                "PurePursuit preserves exact path curvature when omega saturates");
}

static void test_safety_interlocks(void)
{
    Mission_Init();
    EXPECT_TRUE(Mission_CalculateScore() == 0, "Score in FSM_IDLE must be 0");

    RobotSensorFrame_t f;
    RobotDriveCmd_t cmd;

    /* 1. Start unified run -> BLE E-STOP -> phai dung khan cap */
    memset(&f, 0, sizeof(f));
    f.dt_s = 0.02f; f.imu_valid = true; f.v_battery = 8.0f; f.btn_short_press = true;
    Mission_Step(&f, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_RUN_O1_O2, "Safety test started in FSM_RUN_O1_O2");

    f.btn_short_press = false; f.ble_cmd_estop = true;
    Mission_Step(&f, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_EMERGENCY_STOP && cmd.brake_stop,
                "BLE STOP immediately triggers FSM_EMERGENCY_STOP + brake");

    /* 2. Tu E-STOP gui lenh BLE RUN:2 phai khoi dong lai duoc ngay */
    f.ble_cmd_estop = false; f.ble_cmd_start = true;
    Mission_Step(&f, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_RUN_O1_O2,
                "BLE RUN:2 recovers directly from FSM_EMERGENCY_STOP");

    /* 3. E-STOP -> bam PA11 phuc hoi ve IDLE -> pin yeu 6.2V phai ngat khan cap */
    f.ble_cmd_start = false; f.ble_cmd_estop = true;
    Mission_Step(&f, &cmd);
    f.ble_cmd_estop = false; f.btn_short_press = true;
    Mission_Step(&f, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_IDLE, "PA11 recovers from E-STOP to IDLE");

    f.btn_short_press = true; f.v_battery = 7.8f;
    Mission_Step(&f, &cmd);
    f.btn_short_press = false; f.v_battery = 6.2f;
    Mission_Step(&f, &cmd);
    EXPECT_TRUE(Mission_GetState() == FSM_EMERGENCY_STOP && cmd.brake_stop,
                "Critical low LiPo voltage (< 6.6V) triggers safe E-STOP");
}

static void test_noisy_imu_and_slip_stress(void)
{
    SimWorld_t world;
    sim_init_world(&world, true, 125.0f, 30.0f); /* MODE_C + nhieu cam bien */
    Mission_Init();

    RobotSensorFrame_t frame;
    RobotDriveCmd_t cmd = {0};
    memset(&frame, 0, sizeof(frame));
    frame.dt_s = 0.02f;
    frame.imu_valid = true;
    frame.v_battery = 7.9f;
    frame.btn_short_press = true;
    Mission_Step(&frame, &cmd);

    float max_wheel = 0.0f;
    float min_clr = 999.0f;
    uint32_t rng = 1234567U;
    for (int step = 0; step < 2500; step++) {
        sim_step_physics(&world, &cmd, 0.02f, &frame, &max_wheel);
        rng = rng * 1664525U + 1013904223U;
        int noise_mm = (int)((rng >> 16) % 17U) - 8;
        if (frame.tof_front_valid && (int)frame.tof_front_mm + noise_mm > 30) {
            frame.tof_front_mm = (uint16_t)((int)frame.tof_front_mm + noise_mm);
        }
        if (step >= 200 && step < 210) {
            frame.delta_ticks_left = (int32_t)(frame.delta_ticks_left * 0.5f);
        }

        Mission_Step(&frame, &cmd);
        if (world.y >= 0.0f) {
            float clr = min_true_clearance_cm(&world, world.x, world.y);
            if (clr < min_clr) min_clr = clr;
        }
        if (Mission_GetState() == FSM_MISSION_FINISHED ||
            Mission_GetState() == FSM_EMERGENCY_STOP) {
            break;
        }
    }

    RobotPose_t est = Mission_GetPose();
    float err = hypotf(est.x_cm - world.x, est.y_cm - world.y);
    printf("  [Noisy+Slip Stress] State=%d, True=(%.1f,%.1f), EKF_Err=%.2fcm, DynObs=%u, MinClr=%.2fcm, MaxWheel=%.1fcm/s\n",
           (int)Mission_GetState(), world.x, world.y, err,
           (unsigned)ObstacleMapper_GetCount(Mission_GetMapper()), min_clr, max_wheel);

    EXPECT_TRUE(Mission_GetState() == FSM_MISSION_FINISHED, "Noisy+Slip stress test finishes");
    EXPECT_TRUE(world.y <= -5.5f, "Noisy+Slip: entire 10cm robot clears finish line (y <= -5.5cm)");
    EXPECT_TRUE(min_clr >= 5.2f, "Noisy+Slip: zero obstacle collision");
    EXPECT_TRUE(max_wheel <= ROBOT_MAX_WHEEL_SPEED_CMS + 0.05f, "Noisy+Slip: wheel <= 30cm/s");
}

int main(void)
{
    printf("=== Running test_navigation_e2e (Full Closed-Loop 2D Physics Simulation) ===\n");
    test_map_geometry_and_pp_curvature();
    test_unified_consecutive_runs_identical_speed();
    test_tunable_v_target_persists_across_reset();
    run_random_obstacle_scenario("O3_MidGap",        102.0f,  95.0f);
    run_random_obstacle_scenario("MODE_A_GapT13T14", 115.0f, 100.0f);
    run_random_obstacle_scenario("MODE_B_BelowT14",  125.0f,  65.0f);
    run_random_obstacle_scenario("MODE_C_LeftLane",  125.0f,  30.0f);
    run_random_obstacle_scenario("MODE_D_GapT15T17", 155.0f,  25.0f);
    run_random_obstacle_scenario("MODE_E_FinishGate",145.0f,   5.0f);
    run_random_obstacle_scenario("O5_GoalCenter",    150.0f,   5.0f);
    run_random_obstacle_scenario("O5_GateLeft",      132.0f,   5.0f);
    run_random_obstacle_scenario("O5_GateRight",     162.0f,   5.0f);
    run_random_obstacle_scenario("O5_RightWall",     175.0f,  65.0f);
    run_random_obstacle_scenario("MODE_O4_Center",   150.0f,  85.0f);
    test_noisy_imu_and_slip_stress();
    test_safety_interlocks();

    printf("test_navigation_e2e: %d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
