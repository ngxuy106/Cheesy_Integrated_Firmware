/**
 * ============================================================================
 * @file    pure_pursuit.h
 * @brief   Bo dieu khien bam quy dao Pure Pursuit + Pivot cua gap +
 *          Bao hoa 2 banh bao toan do cong tai gioi han 30 cm/s
 * ============================================================================
 */

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <stdint.h>
#include <stdbool.h>
#include "astar_planner.h"
#include "robot_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PP_DEFAULT_LOOKAHEAD_CM     0.0f    /* 0.0f = Adaptive Lookahead */
#define PP_MIN_LOOKAHEAD_CM         7.5f
#define PP_MAX_LOOKAHEAD_CM         14.5f
#define PP_LOOKAHEAD_GAIN           0.22f
#define PP_LOOKAHEAD_OFFSET_CM      6.5f
#define PP_GOAL_REACHED_DIST_CM     2.0f
#define PP_PIVOT_ENTER_RAD          1.082f  /* 62 deg: xoay tai cho truoc khi cua gap */
#define PP_PIVOT_EXIT_RAD           0.524f  /* 30 deg: tiep tuc tien                  */

typedef struct {
    float lookahead_dist;
    float base_speed_cms;
    int   current_wp_idx;
    bool  is_finished;
    bool  use_adaptive;
    bool  is_pivoting;
    float current_lookahead;
    float current_curvature;
    float current_alpha_rad;
    float target_x_cm;
    float target_y_cm;
} PurePursuit_t;

typedef struct {
    float v_linear_cms;
    float omega_rads;
    float v_left_cms;
    float v_right_cms;
    bool  is_pivoting;
} PurePursuit_Cmd_t;

void PurePursuit_Init(float lookahead_cm, float base_speed_cms);
void PurePursuit_Reset(void);
void PurePursuit_SetBaseSpeed(float base_speed_cms);
void PurePursuit_SetLookahead(float lookahead_cm);
float PurePursuit_CalculateAdaptiveLookahead(float v_target_cms);

/* Tinh lenh dieu khien 1 buoc; tra ve false neu da ve dich hoac loi dau vao */
bool PurePursuit_Compute(RobotPose_t pose,
                         const Waypoint_t *waypoints,
                         int wp_count,
                         PurePursuit_Cmd_t *out_cmd);

bool PurePursuit_IsFinished(void);
PurePursuit_t PurePursuit_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* PURE_PURSUIT_H */
