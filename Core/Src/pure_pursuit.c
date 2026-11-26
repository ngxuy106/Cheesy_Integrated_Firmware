/**
 * ============================================================================
 * @file    pure_pursuit.c
 * @brief   Hien thuc Pure Pursuit thich ung + Pivot hysteresis + gioi han 30cm/s
 * ============================================================================
 */

#include "pure_pursuit.h"
#include "arena_map.h"
#include "robot_config.h"
#include "runtime_config.h"
#include <math.h>
#include <string.h>

static PurePursuit_t s_pp = {
    .lookahead_dist    = 0.0f,
    .base_speed_cms    = 18.0f,
    .current_wp_idx    = 0,
    .is_finished       = false,
    .use_adaptive      = true,
    .is_pivoting       = false,
    .current_lookahead = 13.1f,
    .current_curvature = 0.0f,
    .current_alpha_rad = 0.0f,
    .target_x_cm       = 0.0f,
    .target_y_cm       = 0.0f
};

static float s_active_start_x  = -9999.0f;
static float s_active_start_y  = -9999.0f;
static float s_active_goal_x   = -9999.0f;
static float s_active_goal_y   = -9999.0f;
static float s_active_wp_sum   = -9999.0f;
static int   s_active_wp_count = 0;

float PurePursuit_CalculateAdaptiveLookahead(float v_target_cms)
{
    float ld_offset = (g_cfg.pp_lookahead_offset_cm > 1.0f) ? g_cfg.pp_lookahead_offset_cm : PP_LOOKAHEAD_OFFSET_CM;
    float ld = PP_LOOKAHEAD_GAIN * fabsf(v_target_cms) + ld_offset;
    if (ld < PP_MIN_LOOKAHEAD_CM) ld = PP_MIN_LOOKAHEAD_CM;
    if (ld > PP_MAX_LOOKAHEAD_CM) ld = PP_MAX_LOOKAHEAD_CM;
    return ld;
}

void PurePursuit_Init(float lookahead_cm, float base_speed_cms)
{
    float v_max = (g_cfg.v_max > 1.0f) ? g_cfg.v_max : ROBOT_MAX_SPEED_CMS;
    if (base_speed_cms > v_max) base_speed_cms = v_max;
    if (base_speed_cms < 0.0f) base_speed_cms = 0.0f;

    s_pp.base_speed_cms    = base_speed_cms;
    s_pp.current_wp_idx    = 0;
    s_pp.is_finished       = false;
    s_pp.is_pivoting       = false;
    s_pp.current_curvature = 0.0f;
    s_pp.current_alpha_rad = 0.0f;
    s_active_start_x       = -9999.0f;
    s_active_start_y       = -9999.0f;
    s_active_goal_x        = -9999.0f;
    s_active_goal_y        = -9999.0f;
    s_active_wp_sum        = -9999.0f;
    s_active_wp_count      = 0;

    PurePursuit_SetLookahead(lookahead_cm);
}

void PurePursuit_SetLookahead(float lookahead_cm)
{
    if (lookahead_cm > 0.0f) {
        s_pp.lookahead_dist    = lookahead_cm;
        s_pp.use_adaptive      = false;
        s_pp.current_lookahead = lookahead_cm;
    } else {
        s_pp.lookahead_dist    = 0.0f;
        s_pp.use_adaptive      = true;
        s_pp.current_lookahead = PurePursuit_CalculateAdaptiveLookahead(s_pp.base_speed_cms);
    }
}

void PurePursuit_SetBaseSpeed(float base_speed_cms)
{
    float v_max = (g_cfg.v_max > 1.0f) ? g_cfg.v_max : ROBOT_MAX_SPEED_CMS;
    if (base_speed_cms > v_max) base_speed_cms = v_max;
    if (base_speed_cms < 0.0f) base_speed_cms = 0.0f;
    s_pp.base_speed_cms = base_speed_cms;
    if (s_pp.use_adaptive) {
        s_pp.current_lookahead = PurePursuit_CalculateAdaptiveLookahead(base_speed_cms);
    }
}

void PurePursuit_Reset(void)
{
    s_pp.current_wp_idx    = 0;
    s_pp.is_finished       = false;
    s_pp.is_pivoting       = false;
    s_pp.current_curvature = 0.0f;
    s_pp.current_alpha_rad = 0.0f;
    s_active_start_x       = -9999.0f;
    s_active_start_y       = -9999.0f;
    s_active_goal_x        = -9999.0f;
    s_active_goal_y        = -9999.0f;
    s_active_wp_sum        = -9999.0f;
    s_active_wp_count      = 0;
}

bool PurePursuit_IsFinished(void)
{
    return s_pp.is_finished;
}

PurePursuit_t PurePursuit_GetStatus(void)
{
    return s_pp;
}

static void find_lookahead_target(RobotPose_t pose, const Waypoint_t *waypoints,
                                  int wp_count, float L_d,
                                  float *out_xt, float *out_yt)
{
    if (wp_count <= 1) {
        *out_xt = (wp_count == 1) ? waypoints[0].x : pose.x_cm;
        *out_yt = (wp_count == 1) ? waypoints[0].y : pose.y_cm;
        return;
    }

    if (s_pp.current_wp_idx > wp_count - 2) s_pp.current_wp_idx = wp_count - 2;
    if (s_pp.current_wp_idx < 0) s_pp.current_wp_idx = 0;

    /* Bo qua cac doan xe da di qua hoan toan */
    while (s_pp.current_wp_idx < wp_count - 2) {
        float seg_dx = waypoints[s_pp.current_wp_idx + 1].x - waypoints[s_pp.current_wp_idx].x;
        float seg_dy = waypoints[s_pp.current_wp_idx + 1].y - waypoints[s_pp.current_wp_idx].y;
        float p_dx   = pose.x_cm - waypoints[s_pp.current_wp_idx + 1].x;
        float p_dy   = pose.y_cm - waypoints[s_pp.current_wp_idx + 1].y;
        if ((p_dx * seg_dx + p_dy * seg_dy) >= 0.0f) {
            s_pp.current_wp_idx++;
        } else {
            break;
        }
    }

    /* Neu da o doan cuoi va cach dich <= L_d, di thang vao dich (khong chon giao diem phia sau) */
    if (s_pp.current_wp_idx >= wp_count - 2) {
        float gdx = waypoints[wp_count - 1].x - pose.x_cm;
        float gdy = waypoints[wp_count - 1].y - pose.y_cm;
        if ((gdx * gdx + gdy * gdy) <= (L_d * L_d)) {
            *out_xt = waypoints[wp_count - 1].x;
            *out_yt = waypoints[wp_count - 1].y;
            return;
        }
    }

    /* Tim giao diem xa nhat PHIA TRUOC (t >= min_t) cua vong tron L_d voi cac doan quy dao */
    bool found = false;
    float best_xt = 0.0f, best_yt = 0.0f;
    int search_end = s_pp.current_wp_idx + 4;
    if (search_end > wp_count - 1) search_end = wp_count - 1;

    for (int i = s_pp.current_wp_idx; i < search_end; i++) {
        float ax = waypoints[i].x,     ay = waypoints[i].y;
        float bx = waypoints[i + 1].x, by = waypoints[i + 1].y;
        float vx = bx - ax, vy = by - ay;
        float dx = ax - pose.x_cm, dy = ay - pose.y_cm;

        float a = vx * vx + vy * vy;
        if (a < 1e-4f) continue;

        /* Hinh chieu cua xe len doan hien tai: chi nhan giao diem phia truoc xe */
        float min_t = 0.0f;
        if (i == s_pp.current_wp_idx) {
            float t_proj = ((pose.x_cm - ax) * vx + (pose.y_cm - ay) * vy) / a;
            if (t_proj > 0.0f) min_t = (t_proj < 1.0f) ? t_proj : 1.0f;
        }

        float b = dx * vx + dy * vy;
        float c = dx * dx + dy * dy - L_d * L_d;
        float disc = b * b - a * c;

        if (disc >= 0.0f) {
            float sqrt_disc = sqrtf(disc);
            float t2 = (-b + sqrt_disc) / a;
            float t1 = (-b - sqrt_disc) / a;
            float t = -1.0f;
            if (t2 >= min_t && t2 <= 1.0f) {
                t = t2;
            } else if (t1 >= min_t && t1 <= 1.0f) {
                t = t1;
            }
            if (t >= 0.0f) {
                best_xt = ax + t * vx;
                best_yt = ay + t * vy;
                found = true;
            }
        }
    }

    if (found) {
        *out_xt = best_xt;
        *out_yt = best_yt;
    } else {
        int target_idx = (s_pp.current_wp_idx < wp_count - 1) ? (s_pp.current_wp_idx + 1) : (wp_count - 1);
        *out_xt = waypoints[target_idx].x;
        *out_yt = waypoints[target_idx].y;
    }
}

bool PurePursuit_Compute(RobotPose_t pose,
                         const Waypoint_t *waypoints,
                         int wp_count,
                         PurePursuit_Cmd_t *out_cmd)
{
    if (out_cmd != NULL) {
        memset(out_cmd, 0, sizeof(*out_cmd));
    }
    if (waypoints == NULL || wp_count <= 0 || out_cmd == NULL) {
        return false;
    }

    /* Tu dong reset chi so khi quy dao thay doi sau A* replan (ke ca khi cung so diem) */
    float wp_sum = 0.0f;
    for (int i = 0; i < wp_count; i++) {
        wp_sum += waypoints[i].x * (float)(i + 1) + waypoints[i].y * (float)(i + 3);
    }
    if (wp_count != s_active_wp_count ||
        fabsf(wp_sum - s_active_wp_sum) > 1e-2f ||
        fabsf(waypoints[0].x - s_active_start_x) > 1e-2f ||
        fabsf(waypoints[0].y - s_active_start_y) > 1e-2f ||
        fabsf(waypoints[wp_count - 1].x - s_active_goal_x) > 1e-2f ||
        fabsf(waypoints[wp_count - 1].y - s_active_goal_y) > 1e-2f) {
        s_active_wp_count   = wp_count;
        s_active_wp_sum     = wp_sum;
        s_active_start_x    = waypoints[0].x;
        s_active_start_y    = waypoints[0].y;
        s_active_goal_x     = waypoints[wp_count - 1].x;
        s_active_goal_y     = waypoints[wp_count - 1].y;
        s_pp.current_wp_idx = 0;
        s_pp.is_finished    = false;
        s_pp.is_pivoting    = false;
    }

    /* 1. Kiem tra dieu kien cham dich kep */
    if (Map_IsFinishLineCrossed(pose.x_cm, pose.y_cm)) {
        s_pp.is_finished = true;
        return false;
    }

    float dx_final = waypoints[wp_count - 1].x - pose.x_cm;
    float dy_final = waypoints[wp_count - 1].y - pose.y_cm;
    float dist_final = sqrtf(dx_final * dx_final + dy_final * dy_final);
    if (dist_final < PP_GOAL_REACHED_DIST_CM) {
        s_pp.is_finished = true;
        return false;
    }

    if (s_pp.is_finished) {
        return false;
    }

    /* 2. Khoang cach nhin truoc thich ung L_d */
    float L_d = s_pp.use_adaptive
                    ? PurePursuit_CalculateAdaptiveLookahead(s_pp.base_speed_cms)
                    : s_pp.lookahead_dist;
    if (L_d < PP_MIN_LOOKAHEAD_CM) L_d = PP_MIN_LOOKAHEAD_CM;
    if (L_d > PP_MAX_LOOKAHEAD_CM) L_d = PP_MAX_LOOKAHEAD_CM;
    s_pp.current_lookahead = L_d;

    /* 3. Tim diem muc tieu & chuyen ve he quy chieu than xe */
    float xt = 0.0f, yt = 0.0f;
    find_lookahead_target(pose, waypoints, wp_count, L_d, &xt, &yt);
    s_pp.target_x_cm = xt;
    s_pp.target_y_cm = yt;

    float dx = xt - pose.x_cm;
    float dy = yt - pose.y_cm;
    float cos_th = cosf(pose.theta_rad);
    float sin_th = sinf(pose.theta_rad);
    float dx_local =  dx * cos_th + dy * sin_th;
    float dy_local = -dx * sin_th + dy * cos_th;

    float alpha = atan2f(dy_local, dx_local);
    s_pp.current_alpha_rad = alpha;

    /* 4. Che do xoay tai cho (Pivot) khi lech huong lon (vd sau khi A* replan) */
    if (!s_pp.is_pivoting && fabsf(alpha) > PP_PIVOT_ENTER_RAD) {
        s_pp.is_pivoting = true;
    } else if (s_pp.is_pivoting && fabsf(alpha) < PP_PIVOT_EXIT_RAD) {
        s_pp.is_pivoting = false;
    }

    float track_w = (g_cfg.track_width_cm > 1.0f) ? g_cfg.track_width_cm : ROBOT_TRACK_WIDTH_CM;
    float v_max   = (g_cfg.v_max > 1.0f) ? g_cfg.v_max : ROBOT_MAX_SPEED_CMS;

    if (s_pp.is_pivoting) {
        float w = 2.2f * alpha;
        if (w > ROBOT_PIVOT_OMEGA_RADS)  w =  ROBOT_PIVOT_OMEGA_RADS;
        if (w < -ROBOT_PIVOT_OMEGA_RADS) w = -ROBOT_PIVOT_OMEGA_RADS;
        float half_wb = track_w * 0.5f;

        s_pp.current_curvature = 0.0f;
        out_cmd->v_linear_cms  = 0.0f;
        out_cmd->omega_rads    = w;
        out_cmd->v_left_cms    = -w * half_wb;
        out_cmd->v_right_cms   =  w * half_wb;
        out_cmd->is_pivoting   = true;
        return true;
    }

    /* 5. Tinh do cong chinh xac theo khoang cach thuc toi diem lookahead */
    float dist2 = dx_local * dx_local + dy_local * dy_local;
    if (dist2 < 25.0f) dist2 = 25.0f; /* >= 5cm^2 tranh chia nho gan dich */
    float curvature = (2.0f * dy_local) / dist2;
    s_pp.current_curvature = curvature;

    /* 6. Dieu tiet van toc theo goc cua & gioi han cung v_max */
    float turn_scale = cosf(alpha);
    if (turn_scale < 0.55f) turn_scale = 0.55f;
    float v_cmd = s_pp.base_speed_cms * turn_scale;
    if (v_cmd > v_max) v_cmd = v_max;
    if (v_cmd < ROBOT_MIN_CRUISE_SPEED_CMS && s_pp.base_speed_cms >= ROBOT_MIN_CRUISE_SPEED_CMS) {
        v_cmd = ROBOT_MIN_CRUISE_SPEED_CMS;
    }

    /* Bao toan dung do cong (ban kinh cua) khi omega cham nguong ROBOT_MAX_OMEGA_RADS */
    float omega = v_cmd * curvature;
    if (fabsf(omega) > ROBOT_MAX_OMEGA_RADS) {
        float scale = ROBOT_MAX_OMEGA_RADS / fabsf(omega);
        v_cmd *= scale;
        omega *= scale;
    }

    float half_wb = track_w * 0.5f;
    float v_left  = v_cmd - omega * half_wb;
    float v_right = v_cmd + omega * half_wb;

    /* 7. Bao hoa van toc banh bao toan do cong: max(|v_L|, |v_R|) <= v_max */
    float peak = fmaxf(fabsf(v_left), fabsf(v_right));
    if (peak > v_max) {
        float scale = v_max / peak;
        v_left  *= scale;
        v_right *= scale;
        v_cmd    = 0.5f * (v_left + v_right);
        omega    = (v_right - v_left) / track_w;
    }

    out_cmd->v_linear_cms = v_cmd;
    out_cmd->omega_rads   = omega;
    out_cmd->v_left_cms   = v_left;
    out_cmd->v_right_cms  = v_right;
    out_cmd->is_pivoting  = false;
    return true;
}
