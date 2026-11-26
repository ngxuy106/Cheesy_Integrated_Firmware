/**
 * ============================================================================
 * @file    astar_planner.h
 * @brief   Bo tim duong A* 8 huong chong cat goc + lam muot Raymarching +
 *          cap nhat vat can dong 10x10cm tren luoi C-Space 50x74
 * ============================================================================
 */

#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "arena_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_WAYPOINTS       36
#define ASTAR_MAX_QUEUE     1536

typedef struct {
    float x;    /* Toa do X thuc te (cm) */
    float y;    /* Toa do Y thuc te (cm) */
} Waypoint_t;

/* Khoi tao / Dat lai C-Space RAM tu ban do tinh Flash ROM */
void AStar_ResetCSpace(void);

/* Them vat can dong hinh hop (tam cx, cy, nua canh half_size_cm, noi rong inflate_cm) */
void AStar_AddDynamicObstacleBox(float center_x, float center_y,
                                 float half_size_cm, float inflate_cm);

/* Them vat can dong mac dinh 10x10cm + noi rong C-Space */
void AStar_AddDynamicObstacle(float obs_x, float obs_y);

/* Tim duong nghiem ngat (Start & Goal phai nam trong o CELL_FREE) */
int AStar_PlanPath(float start_x, float start_y, float goal_x, float goal_y,
                   Waypoint_t *out_wps, int max_wps);

/* Tim duong cho Replan khi dang chay: neu xe hoi lem vao vung dem CELL_INFLATED,
 * tu dong keo diem bat dau ra o CELL_FREE gan nhat trong ban kinh <= 10cm */
int AStar_PlanPathSafe(float start_x, float start_y, float goal_x, float goal_y,
                       Waypoint_t *out_wps, int max_wps, bool allow_start_snap);

/* Kiem tra 1 diem trong C-Space dong hien tai co trong khong */
bool AStar_IsDynamicFree(float x_cm, float y_cm);

/* Kiem tra quy dao con lai tu from_idx co bi chan boi vat can dong moi nap khong */
bool AStar_IsPathBlocked(const Waypoint_t *wps, int wp_count, int from_idx);

#ifdef __cplusplus
}
#endif

#endif /* ASTAR_PLANNER_H */
