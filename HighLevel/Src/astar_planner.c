/**
 * ============================================================================
 * @file    astar_planner.c
 * @brief   Hien thuc A* 8 huong + soft centerline preference + LoS smoothing
 *          (100% static .bss RAM, zero malloc, an toan stack tren STM32F411)
 * ============================================================================
 */

#include "astar_planner.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

#define ASTAR_MAX_RAW           320
#define ASTAR_WALL_NEAR_PENALTY 0.35f

typedef struct {
    int16_t r;
    int16_t c;
    float   f;
} HeapNode_t;

/* Toan bo bo dem tinh trong .bss (khong cap phat heap, khong chiem stack) */
static uint8_t    s_cspace_grid[MAP_ROWS][MAP_COLS];
static bool       s_cspace_inited = false;
static HeapNode_t s_heap[ASTAR_MAX_QUEUE];
static int        s_heap_size = 0;
static float      s_g_score[MAP_ROWS][MAP_COLS];
static int8_t     s_parent_r[MAP_ROWS][MAP_COLS];
static int8_t     s_parent_c[MAP_ROWS][MAP_COLS];
static uint8_t    s_closed[MAP_ROWS][MAP_COLS];

static int16_t    s_raw_r[ASTAR_MAX_RAW];
static int16_t    s_raw_c[ASTAR_MAX_RAW];
static int16_t    s_fwd_r[ASTAR_MAX_RAW];
static int16_t    s_fwd_c[ASTAR_MAX_RAW];

static const int8_t DIR_R[8] = { -1,  1,  0,  0, -1, -1,  1,  1 };
static const int8_t DIR_C[8] = {  0,  0, -1,  1, -1,  1, -1,  1 };
static const float  DIR_W[8] = {
    1.0f, 1.0f, 1.0f, 1.0f,
    1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f
};

static bool world_to_grid(float x, float y, int *out_r, int *out_c)
{
    if (!isfinite(x) || !isfinite(y)) {
        return false;
    }

    if (x >= -2.5f && x <= 182.5f && y >= -2.5f && y <= 122.5f) {
        int c = (int)floorf((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
        int r = (int)floorf((y + 2.5f + 1e-3f) / GRID_SIZE_CM);
        if (c >= MAP_COLS) c = MAP_COLS - 1;
        if (r >= MAP_ROWS) r = MAP_ROWS - 1;
        if (r >= 0 && r < MAP_ROWS && c >= 0 && c < MAP_COLS) {
            *out_r = r;
            *out_c = c;
            return true;
        }
        return false;
    }

    /* Cong xuat phat O 1: Y in [-10.5, -2.5) */
    if (y >= -10.5f && y < -2.5f && x >= 0.0f && x <= 60.0f) {
        int c = (int)floorf((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
        if (c >= 0 && c < MAP_COLS) {
            *out_r = 0;
            *out_c = c;
            return true;
        }
        return false;
    }

    /* Cong ve dich O 5: Y in [-10.5, -2.5) */
    if (y >= -10.5f && y < -2.5f && x >= 120.0f && x <= 180.0f) {
        int c = (int)floorf((x + 2.5f + 1e-3f) / GRID_SIZE_CM);
        if (c >= 0 && c < MAP_COLS) {
            *out_r = 0;
            *out_c = c;
            return true;
        }
        return false;
    }

    return false;
}

static inline bool heap_push(int16_t r, int16_t c, float f)
{
    if (s_heap_size >= ASTAR_MAX_QUEUE) {
        return false;
    }
    int idx = s_heap_size++;
    s_heap[idx].r = r;
    s_heap[idx].c = c;
    s_heap[idx].f = f;
    while (idx > 0) {
        int parent = (idx - 1) / 2;
        if (s_heap[idx].f < s_heap[parent].f) {
            HeapNode_t tmp = s_heap[idx];
            s_heap[idx] = s_heap[parent];
            s_heap[parent] = tmp;
            idx = parent;
        } else {
            break;
        }
    }
    return true;
}

static inline bool heap_pop(int16_t *r, int16_t *c, float *f)
{
    if (s_heap_size <= 0) {
        return false;
    }
    *r = s_heap[0].r;
    *c = s_heap[0].c;
    *f = s_heap[0].f;
    s_heap_size--;
    if (s_heap_size > 0) {
        s_heap[0] = s_heap[s_heap_size];
        int idx = 0;
        while (1) {
            int left = 2 * idx + 1;
            int right = 2 * idx + 2;
            int smallest = idx;
            if (left < s_heap_size && s_heap[left].f < s_heap[smallest].f) smallest = left;
            if (right < s_heap_size && s_heap[right].f < s_heap[smallest].f) smallest = right;
            if (smallest != idx) {
                HeapNode_t tmp = s_heap[idx];
                s_heap[idx] = s_heap[smallest];
                s_heap[smallest] = tmp;
                idx = smallest;
            } else {
                break;
            }
        }
    }
    return true;
}

static inline float heuristic(int r1, int c1, int r2, int c2)
{
    float dr = (float)(r1 - r2);
    float dc = (float)(c1 - c2);
    return sqrtf(dr * dr + dc * dc);
}

static bool is_line_clear(float x1, float y1, float x2, float y2)
{
    if (!s_cspace_inited) {
        return false;
    }
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dist = sqrtf(dx * dx + dy * dy);
    int steps = (int)ceilf(dist / 0.5f);
    if (steps < 2) steps = 2;
    int prev_r = -1, prev_c = -1;
    for (int s = 0; s <= steps; s++) {
        float t = (float)s / (float)steps;
        int r, c;
        if (!world_to_grid(x1 + t * dx, y1 + t * dy, &r, &c)) {
            return false;
        }
        if (s_cspace_grid[r][c] != CELL_FREE) {
            return false;
        }
        /* Chong tia lam muot luon qua khe goc cheo giua 2 o bi chan */
        if (prev_r >= 0 && r != prev_r && c != prev_c) {
            if (s_cspace_grid[prev_r][c] != CELL_FREE ||
                s_cspace_grid[r][prev_c] != CELL_FREE) {
                return false;
            }
        }
        prev_r = r;
        prev_c = c;
    }
    return true;
}

static bool snap_to_nearest_free(int *io_r, int *io_c, int max_radius)
{
    int r0 = *io_r;
    int c0 = *io_c;
    if (s_cspace_grid[r0][c0] == CELL_FREE) {
        return true;
    }
    /* Chi cho phep snap neu dang o vung dem CELL_INFLATED (khong snap xuyen tuong) */
    if (s_cspace_grid[r0][c0] != CELL_INFLATED) {
        return false;
    }
    int best_r = -1, best_c = -1;
    int best_d2 = 999999;
    for (int rad = 1; rad <= max_radius; rad++) {
        for (int dr = -rad; dr <= rad; dr++) {
            for (int dc = -rad; dc <= rad; dc++) {
                int rr = r0 + dr;
                int cc = c0 + dc;
                if (rr < 0 || rr >= MAP_ROWS || cc < 0 || cc >= MAP_COLS) {
                    continue;
                }
                if (s_cspace_grid[rr][cc] == CELL_FREE) {
                    int d2 = dr * dr + dc * dc;
                    if (d2 < best_d2) {
                        best_d2 = d2;
                        best_r = rr;
                        best_c = cc;
                    }
                }
            }
        }
        if (best_r >= 0) {
            *io_r = best_r;
            *io_c = best_c;
            return true;
        }
    }
    return false;
}

static float cell_proximity_penalty(int r, int c)
{
    for (int i = 0; i < 4; i++) {
        int nr = r + DIR_R[i];
        int nc = c + DIR_C[i];
        if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS ||
            s_cspace_grid[nr][nc] != CELL_FREE) {
            return ASTAR_WALL_NEAR_PENALTY;
        }
    }
    return 0.0f;
}

void AStar_ResetCSpace(void)
{
    const uint8_t *static_map = Map_GetGridPtr();
    if (static_map != NULL) {
        memcpy(s_cspace_grid, static_map, sizeof(s_cspace_grid));
        s_cspace_inited = true;
    }
}

void AStar_AddDynamicObstacleBox(float center_x, float center_y,
                                 float half_size_cm, float inflate_cm)
{
    if (!s_cspace_inited) {
        AStar_ResetCSpace();
    }
    if (!s_cspace_inited || !isfinite(center_x) || !isfinite(center_y)) {
        return;
    }
    if (half_size_cm < 0.5f) half_size_cm = 0.5f;
    if (inflate_cm < 0.0f) inflate_cm = 0.0f;

    float x0 = center_x - half_size_cm;
    float x1 = center_x + half_size_cm;
    float y0 = center_y - half_size_cm;
    float y1 = center_y + half_size_cm;

    float reach = half_size_cm + inflate_cm + GRID_SIZE_CM;
    int c_min = Map_GetGridCol(center_x - reach);
    int c_max = Map_GetGridCol(center_x + reach);
    int r_min = Map_GetGridRow(center_y - reach);
    int r_max = Map_GetGridRow(center_y + reach);

    for (int r = r_min; r <= r_max; r++) {
        float wy = Map_GetWorldY(r);
        for (int c = c_min; c <= c_max; c++) {
            float wx = Map_GetWorldX(c);

            float dx = 0.0f;
            float dy = 0.0f;
            if (wx < x0) dx = x0 - wx;
            else if (wx > x1) dx = wx - x1;

            if (wy < y0) dy = y0 - wy;
            else if (wy > y1) dy = wy - y1;

            float d = sqrtf(dx * dx + dy * dy);
            if (d <= 0.001f) {
                s_cspace_grid[r][c] = CELL_OBST;
            } else if (d <= inflate_cm && s_cspace_grid[r][c] == CELL_FREE) {
                s_cspace_grid[r][c] = CELL_INFLATED;
            }
        }
    }
}

void AStar_AddDynamicObstacle(float obs_x, float obs_y)
{
    AStar_AddDynamicObstacleBox(obs_x, obs_y, TOF_OBS_HALF_SIZE_CM, OBS_DYN_INFLATE_CM);
}

bool AStar_IsDynamicFree(float x_cm, float y_cm)
{
    if (!s_cspace_inited) {
        AStar_ResetCSpace();
    }
    int r, c;
    if (!world_to_grid(x_cm, y_cm, &r, &c)) {
        return false;
    }
    return (s_cspace_grid[r][c] == CELL_FREE);
}

bool AStar_IsPathBlocked(const Waypoint_t *wps, int wp_count, int from_idx)
{
    if (wps == NULL || wp_count < 2 || !s_cspace_inited) {
        return false;
    }
    if (from_idx < 0) from_idx = 0;
    if (from_idx >= wp_count - 1) return false;

    for (int i = from_idx; i < wp_count - 1; i++) {
        if (!is_line_clear(wps[i].x, wps[i].y, wps[i + 1].x, wps[i + 1].y)) {
            return true;
        }
    }
    return false;
}

int AStar_PlanPath(float start_x, float start_y, float goal_x, float goal_y,
                   Waypoint_t *out_wps, int max_wps)
{
    return AStar_PlanPathSafe(start_x, start_y, goal_x, goal_y, out_wps, max_wps, false);
}

int AStar_PlanPathSafe(float start_x, float start_y, float goal_x, float goal_y,
                       Waypoint_t *out_path, int max_waypoints, bool allow_start_snap)
{
    if (out_path == NULL || max_waypoints <= 0) {
        return 0;
    }
    out_path[0].x = 0.0f;
    out_path[0].y = 0.0f;

    if (!s_cspace_inited) {
        AStar_ResetCSpace();
    }
    if (!s_cspace_inited) {
        return 0;
    }

    int sr, sc, gr, gc;
    if (!world_to_grid(start_x, start_y, &sr, &sc) ||
        !world_to_grid(goal_x, goal_y, &gr, &gc)) {
        return 0;
    }

    float eff_start_x = start_x;
    float eff_start_y = start_y;
    if (s_cspace_grid[sr][sc] != CELL_FREE) {
        if (!allow_start_snap || !snap_to_nearest_free(&sr, &sc, 4)) {
            return 0;
        }
        eff_start_x = Map_GetWorldX(sc);
        eff_start_y = Map_GetWorldY(sr);
    }

    if (s_cspace_grid[gr][gc] != CELL_FREE) {
        return 0;
    }

    if (eff_start_x == goal_x && eff_start_y == goal_y) {
        out_path[0] = (Waypoint_t){ goal_x, goal_y };
        return 1;
    }

    if (sr == gr && sc == gc) {
        if (!is_line_clear(eff_start_x, eff_start_y, goal_x, goal_y)) {
            return 0;
        }
        if (max_waypoints < 2) return 0;
        out_path[0] = (Waypoint_t){ eff_start_x, eff_start_y };
        out_path[1] = (Waypoint_t){ goal_x, goal_y };
        return 2;
    }

    if (max_waypoints < 2) {
        return 0;
    }

    for (int r = 0; r < MAP_ROWS; r++) {
        for (int c = 0; c < MAP_COLS; c++) {
            s_g_score[r][c]  = 1e9f;
            s_parent_r[r][c] = -1;
            s_parent_c[r][c] = -1;
            s_closed[r][c]   = 0U;
        }
    }

    s_heap_size = 0;
    s_g_score[sr][sc] = 0.0f;
    if (!heap_push((int16_t)sr, (int16_t)sc, heuristic(sr, sc, gr, gc))) {
        return 0;
    }

    bool found = false;
    while (s_heap_size > 0) {
        int16_t cr, cc;
        float cf;
        if (!heap_pop(&cr, &cc, &cf)) break;
        if (cr == gr && cc == gc) {
            found = true;
            break;
        }
        if (s_closed[cr][cc]) continue;
        s_closed[cr][cc] = 1U;

        float current_g = s_g_score[cr][cc];
        for (int i = 0; i < 8; i++) {
            int16_t nr = (int16_t)(cr + DIR_R[i]);
            int16_t nc = (int16_t)(cc + DIR_C[i]);
            if (nr < 0 || nr >= MAP_ROWS || nc < 0 || nc >= MAP_COLS) continue;
            if (s_cspace_grid[nr][nc] != CELL_FREE) continue;

            /* Chong cat goc qua o vat can khi di cheo */
            if (i >= 4) {
                if (s_cspace_grid[cr + DIR_R[i]][cc] != CELL_FREE ||
                    s_cspace_grid[cr][cc + DIR_C[i]] != CELL_FREE) {
                    continue;
                }
            }

            float tentative_g = current_g + DIR_W[i] + cell_proximity_penalty(nr, nc);
            if (tentative_g < s_g_score[nr][nc]) {
                s_g_score[nr][nc]  = tentative_g;
                s_parent_r[nr][nc] = (int8_t)cr;
                s_parent_c[nr][nc] = (int8_t)cc;
                if (!heap_push(nr, nc, tentative_g + heuristic(nr, nc, gr, gc))) {
                    return 0;
                }
            }
        }
    }

    if (!found) {
        return 0;
    }

    /* Truy vet nguoc Goal -> Start */
    int raw_count = 0;
    int16_t tr = (int16_t)gr, tc = (int16_t)gc;
    while (raw_count < ASTAR_MAX_RAW) {
        if (tr < 0 || tr >= MAP_ROWS || tc < 0 || tc >= MAP_COLS) {
            return 0;
        }
        s_raw_r[raw_count] = tr;
        s_raw_c[raw_count] = tc;
        raw_count++;

        if (tr == sr && tc == sc) break;

        int8_t pr = s_parent_r[tr][tc];
        int8_t pc = s_parent_c[tr][tc];
        if (pr < 0 || pc < 0) return 0;
        tr = pr;
        tc = pc;
    }

    if (raw_count == 0 || s_raw_r[raw_count - 1] != sr || s_raw_c[raw_count - 1] != sc) {
        return 0;
    }

    for (int i = 0; i < raw_count; i++) {
        s_fwd_r[i] = s_raw_r[raw_count - 1 - i];
        s_fwd_c[i] = s_raw_c[raw_count - 1 - i];
    }

    /* Lam muot Raymarching Line-of-Sight */
    int smooth_count = 0;
    out_path[smooth_count++] = (Waypoint_t){ eff_start_x, eff_start_y };
    int current_idx = 0;
    float curr_x = eff_start_x;
    float curr_y = eff_start_y;

    while (current_idx < raw_count - 1) {
        if (is_line_clear(curr_x, curr_y, goal_x, goal_y)) {
            if (smooth_count >= max_waypoints) return 0;
            out_path[smooth_count++] = (Waypoint_t){ goal_x, goal_y };
            break;
        }

        int furthest_idx = current_idx + 1;
        for (int next_idx = raw_count - 1; next_idx > current_idx; next_idx--) {
            float nx = (next_idx == raw_count - 1) ? goal_x : Map_GetWorldX(s_fwd_c[next_idx]);
            float ny = (next_idx == raw_count - 1) ? goal_y : Map_GetWorldY(s_fwd_r[next_idx]);
            if (is_line_clear(curr_x, curr_y, nx, ny)) {
                furthest_idx = next_idx;
                break;
            }
        }

        current_idx = furthest_idx;
        if (current_idx == raw_count - 1) {
            if (smooth_count >= max_waypoints) return 0;
            out_path[smooth_count++] = (Waypoint_t){ goal_x, goal_y };
            break;
        } else {
            if (smooth_count >= max_waypoints - 1) return 0;
            curr_x = Map_GetWorldX(s_fwd_c[current_idx]);
            curr_y = Map_GetWorldY(s_fwd_r[current_idx]);
            out_path[smooth_count++] = (Waypoint_t){ curr_x, curr_y };
        }
    }

    if (smooth_count < 2 ||
        out_path[smooth_count - 1].x != goal_x ||
        out_path[smooth_count - 1].y != goal_y) {
        return 0;
    }

    return smooth_count;
}
