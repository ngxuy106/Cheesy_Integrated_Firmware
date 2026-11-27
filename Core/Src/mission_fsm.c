/**
 * ============================================================================
 * @file    mission_fsm.c
 * @brief   Hien thuc May trang thai tu hanh hop nhat 1 che do duy nhat:
 *          chay O 1..O 2 -> bat 3x VL53L0X quet map tu O 3..O 5 + A* replan
 *          + Pure Pursuit <= 30 cm/s + v_target hieu chinh duoc sau moi lan test
 * ============================================================================
 */

#include "mission_fsm.h"
#include "arena_map.h"
#include "pure_pursuit.h"
#include "robot_config.h"
#include "runtime_config.h"
#include <math.h>
#include <string.h>

static FSMState_t       s_state               = FSM_IDLE;
static FSMState_t       s_prev_run_state      = FSM_RUN_O1_O2;
static EKF_Localizer_t  s_ekf;
static ObstacleMapper_t s_mapper;
static Waypoint_t       s_path[MAX_WAYPOINTS];
static Waypoint_t       s_plan_buf[MAX_WAYPOINTS];
static int              s_total_wps           = 0;
static SegmentProfile_t s_profiles[TRACK_SEG_COUNT];

static uint32_t         s_elapsed_ms          = 0U;
static uint32_t         s_stall_timer_ms      = 0U;
static uint32_t         s_recovery_timer_ms   = 0U;
static float            s_recovery_start_x    = 0.0f;
static float            s_recovery_start_y    = 0.0f;
static uint8_t          s_recovery_count      = 0U;
static TrackSegment_t   s_max_seg             = TRACK_SEG_O1_START_NARROW;
static bool             s_run_started         = false;
static float            s_active_v_target_cms = V_TARGET_O1_CMS;
static bool             s_replan_done         = false;
static float            s_dyn_inflate_cm      = OBS_DYN_INFLATE_SAFE_CM;
static bool             s_has_pending_obs     = false;
static float            s_pending_obs_x       = 0.0f;
static float            s_pending_obs_y       = 0.0f;

static float clamp_v_target(float v_cms)
{
    float v_max = (g_cfg.v_max > 1.0f) ? g_cfg.v_max : ROBOT_MAX_SPEED_CMS;
    if (v_cms < ROBOT_MIN_CRUISE_SPEED_CMS) return ROBOT_MIN_CRUISE_SPEED_CMS;
    if (v_cms > v_max)                      return v_max;
    return v_cms;
}

/* Quy hoach den cua dich O 5 (x in [128..172], y = -8cm):
 * Luon chon cua dich (tam 150, trai 130, phai 166) co duong di ngan nhat & an toan nhat
 * (tu dong ne coc dat ngay cua dich nhu MODE_E (145,5), (150,5), (132,5) ma khong quay dau 180). */
static float compute_path_len(const Waypoint_t *wps, int n)
{
    float total = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        float dx = wps[i + 1].x - wps[i].x;
        float dy = wps[i + 1].y - wps[i].y;
        total += sqrtf(dx * dx + dy * dy);
    }
    return total;
}

static int plan_to_finish_gate(float start_x, float start_y, bool allow_snap,
                               Waypoint_t *out_path, int max_wps)
{
    static const float k_goals[7][2] = {
        { ROBOT_GOAL_X, ROBOT_GOAL_Y },
        { 130.0f,       ROBOT_GOAL_Y },
        { 166.0f,       ROBOT_GOAL_Y },
        { 138.0f,       ROBOT_GOAL_Y },
        { 158.0f,       ROBOT_GOAL_Y },
        { 129.0f,       ROBOT_GOAL_Y },
        { 170.0f,       ROBOT_GOAL_Y }
    };

    uint8_t dyn_cnt = ObstacleMapper_GetCount(&s_mapper);
    if (dyn_cnt == 0U) {
        return AStar_PlanPathSafe(start_x, start_y,
                                  ROBOT_GOAL_X, ROBOT_GOAL_Y,
                                  out_path, max_wps, allow_snap);
    }

    static Waypoint_t s_cand_path[MAX_WAYPOINTS];
    int   best_wps = 0;
    float best_len = 1e9f;

    for (int k = 0; k < 7; k++) {
        int wps = AStar_PlanPathSafe(start_x, start_y,
                                     k_goals[k][0], k_goals[k][1],
                                     s_cand_path, MAX_WAYPOINTS, allow_snap);
        if (wps >= 2 && wps <= max_wps) {
            float len = compute_path_len(s_cand_path, wps);
            if (k == 0) len -= 4.0f;
            if (len < best_len) {
                best_len = len;
                best_wps = wps;
                memcpy(out_path, s_cand_path, (size_t)wps * sizeof(Waypoint_t));
            }
        }
    }
    return best_wps;
}

static int plan_with_two_tier_inflation(float start_x, float start_y, bool allow_snap,
                                        Waypoint_t *out_path, int max_wps)
{
    uint8_t dyn_cnt = ObstacleMapper_GetCount(&s_mapper);
    if (dyn_cnt == 0U) {
        AStar_ResetCSpace();
        s_dyn_inflate_cm = OBS_DYN_INFLATE_SAFE_CM;
        return plan_to_finish_gate(start_x, start_y, allow_snap, out_path, max_wps);
    }

    AStar_RebuildDynamicCSpace(&s_mapper, OBS_DYN_INFLATE_SAFE_CM);
    int wps = plan_to_finish_gate(start_x, start_y, allow_snap, out_path, max_wps);
    if (wps >= 2) {
        s_dyn_inflate_cm = OBS_DYN_INFLATE_SAFE_CM;
        return wps;
    }

    AStar_RebuildDynamicCSpace(&s_mapper, OBS_DYN_INFLATE_MIN_CM);
    wps = plan_to_finish_gate(start_x, start_y, allow_snap, out_path, max_wps);
    if (wps >= 2) {
        s_dyn_inflate_cm = OBS_DYN_INFLATE_MIN_CM;
    }
    return wps;
}

static bool paths_equal(const Waypoint_t *a, int na, const Waypoint_t *b, int nb)
{
    if (na != nb || na <= 0) {
        return false;
    }
    for (int i = 0; i < na; i++) {
        if (fabsf(a[i].x - b[i].x) > 1e-2f || fabsf(a[i].y - b[i].y) > 1e-2f) {
            return false;
        }
    }
    return true;
}

static void commit_planned_path(const Waypoint_t *new_path, int new_wps, bool reset_pp_if_changed)
{
    bool changed = !paths_equal(s_path, s_total_wps, new_path, new_wps);
    memcpy(s_path, new_path, (size_t)new_wps * sizeof(Waypoint_t));
    s_total_wps = new_wps;
    if (reset_pp_if_changed && changed) {
        PurePursuit_Reset();
    }
}

static void reset_run_diagnostics(void)
{
    for (int i = 0; i < TRACK_SEG_COUNT; i++) {
        s_profiles[i].min_tof_dist_cm = 999.0f;
        s_profiles[i].max_curvature   = 0.0f;
        s_profiles[i].elapsed_ms      = 0U;
    }
}

static void init_default_profiles(void)
{
    if (g_cfg.magic == RUNTIME_CFG_MAGIC) {
        for (int i = 0; i < TRACK_SEG_COUNT; i++) {
            s_profiles[i].v_target_cms = clamp_v_target(g_cfg.v_target[i]);
        }
    } else {
        s_profiles[TRACK_SEG_O1_START_NARROW].v_target_cms = V_TARGET_O1_CMS;
        s_profiles[TRACK_SEG_O2_CORNER].v_target_cms       = V_TARGET_O2_CMS;
        s_profiles[TRACK_SEG_O3_STRAIGHT].v_target_cms     = V_TARGET_O3_CMS;
        s_profiles[TRACK_SEG_O4_CORNER_SCAN].v_target_cms  = V_TARGET_O4_CMS;
        s_profiles[TRACK_SEG_O5_FINISH_GATE].v_target_cms  = V_TARGET_O5_CMS;
    }
    reset_run_diagnostics();
}

static bool start_run(const RobotSensorFrame_t *sensors)
{
    bool imu_ok = (sensors != NULL) && sensors->imu_valid;
    float imu_y = (sensors != NULL) ? sensors->imu_yaw_deg : 0.0f;

    reset_run_diagnostics();
    EKF_ResetPose(&s_ekf, ROBOT_START_X, ROBOT_START_Y, ROBOT_START_YAW, imu_ok, imu_y);
    AStar_ResetCSpace();
    ObstacleMapper_Init(&s_mapper);

    if (s_has_pending_obs) {
        (void)ObstacleMapper_RegisterManual(&s_mapper, s_pending_obs_x, s_pending_obs_y);
        s_has_pending_obs = false;
    }

    s_dyn_inflate_cm = OBS_DYN_INFLATE_SAFE_CM;
    s_total_wps = plan_with_two_tier_inflation(ROBOT_START_X, ROBOT_START_Y, false,
                                               s_path, MAX_WAYPOINTS);
    if (s_total_wps < 2) {
        s_state = FSM_EMERGENCY_STOP;
        return false;
    }

    float v0 = s_profiles[TRACK_SEG_O1_START_NARROW].v_target_cms;
    PurePursuit_Init(PP_DEFAULT_LOOKAHEAD_CM, v0);

    s_elapsed_ms          = 0U;
    s_replan_done         = false;
    s_stall_timer_ms      = 0U;
    s_recovery_count      = 0U;
    s_max_seg             = TRACK_SEG_O1_START_NARROW;
    s_run_started         = true;
    s_active_v_target_cms = v0;
    s_state               = FSM_RUN_O1_O2;
    return true;
}

static void enter_recovery_backup(RobotPose_t pose, uint16_t front_mm, bool register_front_obs)
{
    if (s_recovery_count >= ROBOT_MAX_RECOVERIES_PER_LAP) {
        s_state = FSM_EMERGENCY_STOP;
        return;
    }

    if (register_front_obs) {
        float dist_cm = (front_mm >= TOF_MIN_VALID_MM && front_mm < TOF_OUT_OF_RANGE_MM)
                            ? ((float)front_mm * 0.1f)
                            : 7.0f;
        float cos_th = cosf(pose.theta_rad);
        float sin_th = sinf(pose.theta_rad);
        float hx = pose.x_cm + (TOF_OFFSET_FRONT_CM + dist_cm) * cos_th;
        float hy = pose.y_cm + (TOF_OFFSET_FRONT_CM + dist_cm) * sin_th;
        float cx = hx + TOF_OBS_HALF_SIZE_CM * cos_th;
        float cy = hy + TOF_OBS_HALF_SIZE_CM * sin_th;

        bool not_static = (Map_DistToStaticObstacleCm(hx, hy) > OBS_STATIC_HIT_TOL_CM) &&
                          (Map_DistToStaticObstacleCm(cx, cy) > OBS_STATIC_CENTER_TOL_CM);
        if (not_static) {
            if (ObstacleMapper_RegisterManual(&s_mapper, cx, cy)) {
                AStar_RebuildDynamicCSpace(&s_mapper, s_dyn_inflate_cm);
            }
        }
    }

    s_prev_run_state    = s_state;
    s_recovery_start_x  = pose.x_cm;
    s_recovery_start_y  = pose.y_cm;
    s_recovery_timer_ms = 0U;
    s_recovery_count++;
    s_stall_timer_ms    = 0U;
    s_state             = FSM_RECOVERY_BACKUP;
}

static bool handle_dynamic_replan(RobotPose_t pose, const ObstacleScanResult_t *scan)
{
    if (scan->new_obstacle_found) {
        int new_wps = plan_with_two_tier_inflation(pose.x_cm, pose.y_cm, true,
                                                   s_plan_buf, MAX_WAYPOINTS);
        if (new_wps >= 2) {
            commit_planned_path(s_plan_buf, new_wps, true);
            s_replan_done = true;
            return true;
        }

        enter_recovery_backup(pose, TOF_EMERGENCY_GUARD_MM, false);
        return false;
    }

    if (!scan->obstacle_refined) {
        return true;
    }

    AStar_RebuildDynamicCSpace(&s_mapper, s_dyn_inflate_cm);
    PurePursuit_t pp = PurePursuit_GetStatus();
    if (!AStar_IsRemainingPathBlocked(pose.x_cm, pose.y_cm, s_path, s_total_wps, pp.current_wp_idx)) {
        return true;
    }

    if (s_dyn_inflate_cm > (OBS_DYN_INFLATE_MIN_CM + 0.05f)) {
        int safe_wps = plan_to_finish_gate(pose.x_cm, pose.y_cm, true,
                                           s_plan_buf, MAX_WAYPOINTS);
        if (safe_wps >= 2) {
            commit_planned_path(s_plan_buf, safe_wps, true);
            s_dyn_inflate_cm = OBS_DYN_INFLATE_SAFE_CM;
            s_replan_done    = true;
            return true;
        }

        AStar_RebuildDynamicCSpace(&s_mapper, OBS_DYN_INFLATE_MIN_CM);
        s_dyn_inflate_cm = OBS_DYN_INFLATE_MIN_CM;
        if (!AStar_IsRemainingPathBlocked(pose.x_cm, pose.y_cm, s_path, s_total_wps, pp.current_wp_idx)) {
            return true;
        }
    }

    int min_wps = plan_to_finish_gate(pose.x_cm, pose.y_cm, true,
                                      s_plan_buf, MAX_WAYPOINTS);
    if (min_wps >= 2) {
        commit_planned_path(s_plan_buf, min_wps, true);
        s_dyn_inflate_cm = OBS_DYN_INFLATE_MIN_CM;
        s_replan_done    = true;
        return true;
    }

    enter_recovery_backup(pose, TOF_EMERGENCY_GUARD_MM, false);
    return false;
}

static void record_run_telemetry(TrackSegment_t seg, uint32_t dt_ms, float min_tof_cm)
{
    if ((int)seg < 0 || seg >= TRACK_SEG_COUNT) {
        return;
    }
    s_profiles[seg].elapsed_ms += dt_ms;
    if (min_tof_cm < s_profiles[seg].min_tof_dist_cm) {
        s_profiles[seg].min_tof_dist_cm = min_tof_cm;
    }
    PurePursuit_t pp = PurePursuit_GetStatus();
    float k = fabsf(pp.current_curvature);
    if (k > s_profiles[seg].max_curvature) {
        s_profiles[seg].max_curvature = k;
    }
}

static void output_stop(RobotDriveCmd_t *cmd, bool still)
{
    cmd->v_linear_cms   = 0.0f;
    cmd->omega_rads     = 0.0f;
    cmd->brake_stop     = true;
    cmd->robot_is_still = still;
}

static void reset_common_state(void)
{
    s_state               = FSM_IDLE;
    s_prev_run_state      = FSM_RUN_O1_O2;
    s_total_wps           = 0;
    s_elapsed_ms          = 0U;
    s_stall_timer_ms      = 0U;
    s_recovery_timer_ms   = 0U;
    s_recovery_count      = 0U;
    s_max_seg             = TRACK_SEG_O1_START_NARROW;
    s_run_started         = false;
    s_active_v_target_cms = s_profiles[TRACK_SEG_O1_START_NARROW].v_target_cms;
    s_replan_done         = false;
    s_dyn_inflate_cm      = OBS_DYN_INFLATE_SAFE_CM;
    s_has_pending_obs     = false;
}

void Mission_Init(void)
{
    init_default_profiles();
    reset_common_state();
    s_active_v_target_cms = s_profiles[TRACK_SEG_O1_START_NARROW].v_target_cms;
    EKF_Init(&s_ekf, ROBOT_START_X, ROBOT_START_Y, ROBOT_START_YAW);
    ObstacleMapper_Init(&s_mapper);
    AStar_ResetCSpace();
    PurePursuit_Init(PP_DEFAULT_LOOKAHEAD_CM, s_active_v_target_cms);
}

void Mission_ResetAll(bool imu_valid, float current_imu_yaw_deg)
{
    reset_run_diagnostics();
    reset_common_state();
    EKF_ResetPose(&s_ekf, ROBOT_START_X, ROBOT_START_Y, ROBOT_START_YAW,
                  imu_valid, current_imu_yaw_deg);
    ObstacleMapper_Init(&s_mapper);
    AStar_ResetCSpace();
    PurePursuit_Init(PP_DEFAULT_LOOKAHEAD_CM, s_active_v_target_cms);
}

bool Mission_SetSegmentTargetSpeed(TrackSegment_t seg, float v_target_cms)
{
    if ((int)seg < 0 || seg >= TRACK_SEG_COUNT || !isfinite(v_target_cms)) {
        return false;
    }
    float v = clamp_v_target(v_target_cms);
    s_profiles[seg].v_target_cms = v;
    if (g_cfg.magic == RUNTIME_CFG_MAGIC) {
        g_cfg.v_target[seg] = v;
    }
    return true;
}

void Mission_SetAllTargetSpeeds(float v_o1, float v_o2, float v_o3, float v_o4, float v_o5)
{
    (void)Mission_SetSegmentTargetSpeed(TRACK_SEG_O1_START_NARROW, v_o1);
    (void)Mission_SetSegmentTargetSpeed(TRACK_SEG_O2_CORNER,       v_o2);
    (void)Mission_SetSegmentTargetSpeed(TRACK_SEG_O3_STRAIGHT,     v_o3);
    (void)Mission_SetSegmentTargetSpeed(TRACK_SEG_O4_CORNER_SCAN,  v_o4);
    (void)Mission_SetSegmentTargetSpeed(TRACK_SEG_O5_FINISH_GATE,  v_o5);
}

bool Mission_SetUniformTargetSpeed(float v_target_cms)
{
    if (!isfinite(v_target_cms)) {
        return false;
    }
    float v = clamp_v_target(v_target_cms);
    for (int i = 0; i < TRACK_SEG_COUNT; i++) {
        s_profiles[i].v_target_cms = v;
        if (g_cfg.magic == RUNTIME_CFG_MAGIC) {
            g_cfg.v_target[i] = v;
        }
    }
    return true;
}

float Mission_GetSegmentTargetSpeed(TrackSegment_t seg)
{
    if ((int)seg < 0 || seg >= TRACK_SEG_COUNT) {
        return 0.0f;
    }
    return s_profiles[seg].v_target_cms;
}

bool Mission_InjectObstacle(float obs_x_cm, float obs_y_cm)
{
    if (!isfinite(obs_x_cm) || !isfinite(obs_y_cm)) {
        return false;
    }
    if (s_state == FSM_IDLE) {
        s_has_pending_obs = true;
        s_pending_obs_x   = obs_x_cm;
        s_pending_obs_y   = obs_y_cm;
        return true;
    }

    if (ObstacleMapper_RegisterManual(&s_mapper, obs_x_cm, obs_y_cm)) {
        RobotPose_t pose = EKF_GetPose(&s_ekf);
        int new_wps = plan_with_two_tier_inflation(pose.x_cm, pose.y_cm, true,
                                                   s_plan_buf, MAX_WAYPOINTS);
        if (new_wps >= 2) {
            commit_planned_path(s_plan_buf, new_wps, true);
            s_replan_done = true;
            return true;
        }
    }
    return false;
}

TrackSegment_t Mission_DetectSegment(RobotPose_t pose)
{
    if (pose.x_cm < 60.0f) {
        return (pose.y_cm < 60.0f) ? TRACK_SEG_O1_START_NARROW : TRACK_SEG_O2_CORNER;
    } else if (pose.x_cm < 120.0f) {
        return TRACK_SEG_O3_STRAIGHT;
    } else {
        return (pose.y_cm >= 60.0f) ? TRACK_SEG_O4_CORNER_SCAN : TRACK_SEG_O5_FINISH_GATE;
    }
}

void Mission_Step(const RobotSensorFrame_t *sensors, RobotDriveCmd_t *out_cmd)
{
    if (out_cmd == NULL) {
        return;
    }
    output_stop(out_cmd, true);

    if (sensors == NULL) {
        return;
    }

    float dt_s = (sensors->dt_s > 1e-4f && sensors->dt_s <= 0.5f)
                     ? sensors->dt_s
                     : ROBOT_CTRL_DT_S;
    uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f + 0.5f);
    if (dt_ms == 0U) dt_ms = ROBOT_CTRL_PERIOD_MS;

    /* 1. Luon cap nhat EKF (Encoder Odometry + IMU Gyro Z) */
    EKF_Update(&s_ekf,
               sensors->delta_ticks_left,
               sensors->delta_ticks_right,
               dt_s,
               sensors->imu_valid,
               sensors->imu_yaw_deg,
               sensors->imu_gz_rads);
    RobotPose_t pose = EKF_GetPose(&s_ekf);

    /* 2. Cac tin hieu uu tien cao: Reset / E-Stop / Pin yeu nghiem trong */
    if (sensors->ble_cmd_reset) {
        Mission_ResetAll(sensors->imu_valid, sensors->imu_yaw_deg);
        return;
    }
    if (sensors->ble_cmd_estop) {
        s_state = FSM_EMERGENCY_STOP;
        return;
    }
    if (sensors->v_battery >= ROBOT_VBAT_MIN_VALID_V &&
        sensors->v_battery < ROBOT_VBAT_CRITICAL_V &&
        s_state != FSM_IDLE && s_state != FSM_MISSION_FINISHED) {
        s_state = FSM_EMERGENCY_STOP;
        return;
    }

    /* 3. Dem thoi gian chay tu hanh */
    if (s_state == FSM_RUN_O1_O2 ||
        s_state == FSM_RUN_SCAN_O3_O5 ||
        s_state == FSM_RECOVERY_BACKUP) {
        s_elapsed_ms += dt_ms;
    }

    /* 4. Thuc thi May trang thai FSM hop nhat 1 che do duy nhat */
    switch (s_state) {
        case FSM_IDLE:
            output_stop(out_cmd, true);
            if (sensors->btn_short_press || sensors->btn_long_press || sensors->ble_cmd_start) {
                (void)start_run(sensors);
            }
            break;

        case FSM_RUN_O1_O2:
        case FSM_RUN_SCAN_O3_O5: {
            if (sensors->btn_long_press) {
                s_state = FSM_EMERGENCY_STOP;
                break;
            }

            if (Map_IsFinishLineCrossed(pose.x_cm, pose.y_cm) || PurePursuit_IsFinished()) {
                s_max_seg = TRACK_SEG_O5_FINISH_GATE;
                s_state   = FSM_MISSION_FINISHED;
                output_stop(out_cmd, true);
                break;
            }

            /* Bat 3x VL53L0X quet map tim vat can la ngay khi xe vao vi tri quet (mac dinh O 3) */
            float trig_x = (g_cfg.magic == RUNTIME_CFG_MAGIC) ? g_cfg.scan_trigger_x_cm : FSM_SCAN_TRIGGER_X_CM;
            if (s_state == FSM_RUN_O1_O2 &&
                pose.x_cm >= trig_x &&
                pose.y_cm >= FSM_SCAN_TRIGGER_Y_CM) {
                s_state = FSM_RUN_SCAN_O3_O5;
            }

            bool scan_dyn = (s_state == FSM_RUN_SCAN_O3_O5);
            ObstacleScanResult_t scan =
                ObstacleMapper_Update(&s_mapper, &pose, sensors, scan_dyn, true);

            if (scan.front_emergency) {
                enter_recovery_backup(pose, sensors->tof_front_mm, true);
                output_stop(out_cmd, false);
                break;
            }
            if (scan_dyn && !handle_dynamic_replan(pose, &scan)) {
                output_stop(out_cmd, false);
                break;
            }

            TrackSegment_t seg = Mission_DetectSegment(pose);
            if (seg > s_max_seg) {
                s_max_seg = seg;
            }
            float v_seg = s_profiles[seg].v_target_cms;
            float v_det = (g_cfg.v_detour_max > 1.0f) ? g_cfg.v_detour_max : V_TARGET_DETOUR_MAX_CMS;
            if (s_replan_done && seg >= TRACK_SEG_O3_STRAIGHT && v_seg > v_det) {
                v_seg = v_det;
            }
            s_active_v_target_cms = v_seg;
            PurePursuit_SetBaseSpeed(v_seg);

            if (s_replan_done && seg >= TRACK_SEG_O3_STRAIGHT) {
                PurePursuit_SetLookahead(8.0f);
            } else {
                PurePursuit_SetLookahead(PP_DEFAULT_LOOKAHEAD_CM);
            }

            PurePursuit_Cmd_t pp_cmd;
            if (PurePursuit_Compute(pose, s_path, s_total_wps, &pp_cmd)) {
                out_cmd->v_linear_cms   = pp_cmd.v_linear_cms;
                out_cmd->omega_rads     = pp_cmd.omega_rads;
                out_cmd->brake_stop     = false;
                out_cmd->robot_is_still = false;

                if (!pp_cmd.is_pivoting && fabsf(pp_cmd.v_linear_cms) > 6.0f &&
                    fabsf(pose.v_cms) < 1.0f) {
                    s_stall_timer_ms += dt_ms;
                    if (s_stall_timer_ms >= ROBOT_STALL_TIMEOUT_MS) {
                        enter_recovery_backup(pose, sensors->tof_front_mm, true);
                    }
                } else {
                    s_stall_timer_ms = 0U;
                }
            } else {
                s_max_seg = TRACK_SEG_O5_FINISH_GATE;
                s_state   = FSM_MISSION_FINISHED;
                output_stop(out_cmd, true);
            }

            record_run_telemetry(seg, dt_ms, scan.min_clearance_cm);
            break;
        }

        case FSM_RECOVERY_BACKUP: {
            if (Map_IsFinishLineCrossed(pose.x_cm, pose.y_cm)) {
                s_max_seg = TRACK_SEG_O5_FINISH_GATE;
                s_state   = FSM_MISSION_FINISHED;
                output_stop(out_cmd, true);
                break;
            }

            s_recovery_timer_ms += dt_ms;
            float dx = pose.x_cm - s_recovery_start_x;
            float dy = pose.y_cm - s_recovery_start_y;
            float backed_cm = sqrtf(dx * dx + dy * dy);

            if (backed_cm >= 6.0f || s_recovery_timer_ms >= 700U) {
                int new_wps = plan_with_two_tier_inflation(pose.x_cm, pose.y_cm, true,
                                                           s_plan_buf, MAX_WAYPOINTS);
                if (new_wps >= 2) {
                    commit_planned_path(s_plan_buf, new_wps, true);
                    s_replan_done = true;
                    s_state = s_prev_run_state;
                } else {
                    s_state = FSM_EMERGENCY_STOP;
                }
                output_stop(out_cmd, false);
            } else {
                out_cmd->v_linear_cms   = -12.0f;
                out_cmd->omega_rads     = 0.0f;
                out_cmd->brake_stop     = false;
                out_cmd->robot_is_still = false;
            }
            break;
        }

        case FSM_MISSION_FINISHED:
        case FSM_EMERGENCY_STOP:
        default:
            output_stop(out_cmd, true);
            if (sensors->ble_cmd_start) {
                (void)start_run(sensors);
            } else if (sensors->btn_short_press) {
                Mission_ResetAll(sensors->imu_valid, sensors->imu_yaw_deg);
            }
            break;
    }
}

FSMState_t Mission_GetState(void)
{
    return s_state;
}

RobotPose_t Mission_GetPose(void)
{
    return EKF_GetPose(&s_ekf);
}

const EKF_Localizer_t *Mission_GetEKF(void)
{
    return &s_ekf;
}

const ObstacleMapper_t *Mission_GetMapper(void)
{
    return &s_mapper;
}

const SegmentProfile_t *Mission_GetSegmentProfile(TrackSegment_t seg)
{
    if ((int)seg >= 0 && seg < TRACK_SEG_COUNT) {
        return &s_profiles[seg];
    }
    return NULL;
}

int Mission_GetTotalWaypoints(void)
{
    return s_total_wps;
}

const Waypoint_t *Mission_GetPath(void)
{
    return s_path;
}

int32_t Mission_CalculateScore(void)
{
    if (!s_run_started) {
        return 0;
    }

    static const int32_t k_static_passed_by_seg[TRACK_SEG_COUNT] = {
        0,  /* O 1 */
        4,  /* O 2: T1..T4 */
        8,  /* O 3: T1..T8 */
        13, /* O 4: T1..T13 */
        14  /* O 5: T1..T14 (17 khi ve dich) */
    };

    bool finished = (s_state == FSM_MISSION_FINISHED);
    int32_t static_passed = finished ? 17 : k_static_passed_by_seg[s_max_seg];
    uint32_t t_sec = s_elapsed_ms / 1000U;
    int32_t speed_score = (finished && t_sec < 240U) ? (int32_t)((240U - t_sec) / 6U) : 0;
    int32_t dyn_score   = (finished && s_replan_done) ? 15 : 0;
    int32_t finish_pts  = finished ? 40 : 0;
    int32_t clean_pts   = (finished && s_recovery_count == 0U) ? 10 : 0;
    int32_t score       = (static_passed * 10) + dyn_score + finish_pts + speed_score + clean_pts;
    return (score > 0) ? score : 0;
}

RobotStatus_t Mission_GetStatus(void)
{
    RobotStatus_t st;
    PurePursuit_t pp = PurePursuit_GetStatus();

    st.fsm_state           = (uint8_t)s_state;
    st.active_segment      = (uint8_t)Mission_DetectSegment(EKF_GetPose(&s_ekf));
    st.dynamic_obs_count   = ObstacleMapper_GetCount(&s_mapper);
    st.waypoint_count      = (uint8_t)((s_total_wps > 0) ? s_total_wps : 0);
    st.current_wp_idx      = (uint8_t)((pp.current_wp_idx > 0) ? pp.current_wp_idx : 0);
    st.replan_done         = s_replan_done;
    st.wheel_slip          = s_ekf.wheel_slip_detected;
    st.elapsed_ms          = s_elapsed_ms;
    st.active_v_target_cms = s_active_v_target_cms;
    st.estimated_score     = Mission_CalculateScore();
    return st;
}
