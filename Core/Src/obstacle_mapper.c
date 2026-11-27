/**
 * ============================================================================
 * @file    obstacle_mapper.c
 * @brief   Hien thuc quet map & loc nhieu vat can dong 3 tia VL53L0X
 * ============================================================================
 */

#include "obstacle_mapper.h"
#include "arena_map.h"
#include "runtime_config.h"
#include <math.h>
#include <string.h>

static const float s_ray_angles_rad[3] = {
    TOF_ANGLE_LEFT_DEG  * ROBOT_DEG2RAD_F,
    TOF_ANGLE_FRONT_DEG * ROBOT_DEG2RAD_F,
    TOF_ANGLE_RIGHT_DEG * ROBOT_DEG2RAD_F
};

static const float s_ray_offsets_cm[3] = {
    TOF_OFFSET_LEFT_CM,
    TOF_OFFSET_FRONT_CM,
    TOF_OFFSET_RIGHT_CM
};

typedef struct {
    CarveRay_t ray;
    int8_t     matched_obs_idx;
    bool       has_ray;
} PassFreeRay_t;

static void clamp_arena_center(float *cx, float *cy)
{
    if (*cx < 5.0f)   *cx = 5.0f;
    if (*cx > 175.0f) *cx = 175.0f;
    if (*cy < 5.0f)   *cy = 5.0f;
    if (*cy > 115.0f) *cy = 115.0f;
}

static float dist_point_to_box_cm(float px, float py, float cx, float cy)
{
    float x0 = cx - TOF_OBS_HALF_SIZE_CM;
    float x1 = cx + TOF_OBS_HALF_SIZE_CM;
    float y0 = cy - TOF_OBS_HALF_SIZE_CM;
    float y1 = cy + TOF_OBS_HALF_SIZE_CM;
    float dx = 0.0f;
    float dy = 0.0f;
    if (px < x0) dx = x0 - px;
    else if (px > x1) dx = px - x1;
    if (py < y0) dy = y0 - py;
    else if (py > y1) dy = py - y1;
    return sqrtf(dx * dx + dy * dy);
}

static void clamp_to_feasible_bounds(const DynamicObstacle_t *obs, float *cx, float *cy)
{
    clamp_arena_center(cx, cy);
    if (!obs->has_hit_bbox) {
        return;
    }

    float lo_x = fmaxf(5.0f,   obs->hit_max_x - TOF_OBS_HALF_SIZE_CM);
    float hi_x = fminf(175.0f, obs->hit_min_x + TOF_OBS_HALF_SIZE_CM);
    if (lo_x > hi_x) {
        float m = 0.5f * (lo_x + hi_x);
        lo_x = m;
        hi_x = m;
    }

    float lo_y = fmaxf(5.0f,   obs->hit_max_y - TOF_OBS_HALF_SIZE_CM);
    float hi_y = fminf(115.0f, obs->hit_min_y + TOF_OBS_HALF_SIZE_CM);
    if (lo_y > hi_y) {
        float m = 0.5f * (lo_y + hi_y);
        lo_y = m;
        hi_y = m;
    }

    if (*cx < lo_x) *cx = lo_x;
    if (*cx > hi_x) *cx = hi_x;
    if (*cy < lo_y) *cy = lo_y;
    if (*cy > hi_y) *cy = hi_y;
}

static void clear_ray_cluster(RayCluster_t *rc)
{
    rc->sum_x  = 0.0f;
    rc->sum_y  = 0.0f;
    rc->last_x = 0.0f;
    rc->last_y = 0.0f;
    rc->streak = 0U;
}

static void push_ray_sample(RayCluster_t *rc, float cx, float cy,
                            float hx, float hy, float dx, float dy)
{
    if (rc->streak == 0U) {
        rc->cx[0]     = cx;
        rc->cy[0]     = cy;
        rc->hit_x[0]  = hx;
        rc->hit_y[0]  = hy;
        rc->dir_x[0]  = dx;
        rc->dir_y[0]  = dy;
        rc->sum_x     = cx;
        rc->sum_y     = cy;
        rc->last_x    = cx;
        rc->last_y    = cy;
        rc->streak    = 1U;
        return;
    }

    float cdx = cx - rc->last_x;
    float cdy = cy - rc->last_y;
    if ((cdx * cdx + cdy * cdy) <= (OBS_CLUSTER_RADIUS_CM * OBS_CLUSTER_RADIUS_CM)) {
        uint8_t idx = rc->streak;
        if (idx >= OBS_CONFIRM_FRAMES) {
            for (uint8_t k = 0U; k < (uint8_t)(OBS_CONFIRM_FRAMES - 1U); k++) {
                rc->cx[k]    = rc->cx[k + 1U];
                rc->cy[k]    = rc->cy[k + 1U];
                rc->hit_x[k] = rc->hit_x[k + 1U];
                rc->hit_y[k] = rc->hit_y[k + 1U];
                rc->dir_x[k] = rc->dir_x[k + 1U];
                rc->dir_y[k] = rc->dir_y[k + 1U];
            }
            idx = (uint8_t)(OBS_CONFIRM_FRAMES - 1U);
            rc->streak = (uint8_t)OBS_CONFIRM_FRAMES;
        } else {
            rc->streak++;
        }

        rc->cx[idx]    = cx;
        rc->cy[idx]    = cy;
        rc->hit_x[idx] = hx;
        rc->hit_y[idx] = hy;
        rc->dir_x[idx] = dx;
        rc->dir_y[idx] = dy;
        rc->last_x     = cx;
        rc->last_y     = cy;

        float sx = 0.0f;
        float sy = 0.0f;
        for (uint8_t k = 0U; k < rc->streak; k++) {
            sx += rc->cx[k];
            sy += rc->cy[k];
        }
        rc->sum_x = sx;
        rc->sum_y = sy;
    } else {
        rc->cx[0]     = cx;
        rc->cy[0]     = cy;
        rc->hit_x[0]  = hx;
        rc->hit_y[0]  = hy;
        rc->dir_x[0]  = dx;
        rc->dir_y[0]  = dy;
        rc->sum_x     = cx;
        rc->sum_y     = cy;
        rc->last_x    = cx;
        rc->last_y    = cy;
        rc->streak    = 1U;
    }
}

static void obs_add_hit(DynamicObstacle_t *obs, float hx, float hy, float dx, float dy)
{
    const float max_span = 2.0f * TOF_OBS_HALF_SIZE_CM;
    if (!obs->has_hit_bbox) {
        obs->has_hit_bbox = true;
        obs->hit_min_x    = hx;
        obs->hit_max_x    = hx;
        obs->hit_min_y    = hy;
        obs->hit_max_y    = hy;
    } else {
        /* Kep hit moi trong cua so 10cm quanh bbox hien co de tranh noise keo troi bbox */
        float lo_x = obs->hit_max_x - max_span;
        float hi_x = obs->hit_min_x + max_span;
        float cx_h = hx;
        if (cx_h < lo_x) cx_h = lo_x;
        if (cx_h > hi_x) cx_h = hi_x;
        if (cx_h < obs->hit_min_x) obs->hit_min_x = cx_h;
        if (cx_h > obs->hit_max_x) obs->hit_max_x = cx_h;

        float lo_y = obs->hit_max_y - max_span;
        float hi_y = obs->hit_min_y + max_span;
        float cy_h = hy;
        if (cy_h < lo_y) cy_h = lo_y;
        if (cy_h > hi_y) cy_h = hi_y;
        if (cy_h < obs->hit_min_y) obs->hit_min_y = cy_h;
        if (cy_h > obs->hit_max_y) obs->hit_max_y = cy_h;
    }

    obs->sum_dx += dx;
    obs->sum_dy += dy;
    if (dx > 0.25f)  obs->seen_pos_x = true;
    if (dx < -0.25f) obs->seen_neg_x = true;
    if (dy > 0.25f)  obs->seen_pos_y = true;
    if (dy < -0.25f) obs->seen_neg_y = true;
}

static void estimate_center_from_hits(DynamicObstacle_t *obs)
{
    if (!obs->has_hit_bbox) {
        clamp_arena_center(&obs->base_x_cm, &obs->base_y_cm);
        return;
    }

    float span_x = obs->hit_max_x - obs->hit_min_x;
    float span_y = obs->hit_max_y - obs->hit_min_y;
    float mid_x  = 0.5f * (obs->hit_min_x + obs->hit_max_x);
    float mid_y  = 0.5f * (obs->hit_min_y + obs->hit_max_y);

    float avg_dx = obs->sum_dx;
    float avg_dy = obs->sum_dy;
    float norm   = sqrtf(avg_dx * avg_dx + avg_dy * avg_dy);
    if (norm > 1e-4f) {
        avg_dx /= norm;
        avg_dy /= norm;
    } else {
        avg_dx = 0.0f;
        avg_dy = 0.0f;
    }

    float est_x = mid_x;
    float est_y = mid_y;

    if (span_x >= 4.0f && span_y < 3.5f) {
        est_x = mid_x;
        if (avg_dy < -1e-6f) {
            est_y = mid_y - TOF_OBS_HALF_SIZE_CM;
        } else if (avg_dy > 1e-6f) {
            est_y = mid_y + TOF_OBS_HALF_SIZE_CM;
        } else if (obs->seen_neg_y && !obs->seen_pos_y) {
            est_y = obs->hit_max_y - TOF_OBS_HALF_SIZE_CM;
        } else if (obs->seen_pos_y && !obs->seen_neg_y) {
            est_y = obs->hit_min_y + TOF_OBS_HALF_SIZE_CM;
        } else {
            est_y = mid_y;
        }
    } else if (span_y >= 4.0f && span_x < 3.5f) {
        est_y = mid_y;
        if (avg_dx < -1e-6f) {
            est_x = mid_x - TOF_OBS_HALF_SIZE_CM;
        } else if (avg_dx > 1e-6f) {
            est_x = mid_x + TOF_OBS_HALF_SIZE_CM;
        } else if (obs->seen_neg_x && !obs->seen_pos_x) {
            est_x = obs->hit_max_x - TOF_OBS_HALF_SIZE_CM;
        } else if (obs->seen_pos_x && !obs->seen_neg_x) {
            est_x = obs->hit_min_x + TOF_OBS_HALF_SIZE_CM;
        } else {
            est_x = mid_x;
        }
    } else if (span_x >= 3.5f && span_y >= 3.5f) {
        if ((obs->seen_pos_x && obs->seen_neg_x) || (span_x >= 8.0f)) {
            est_x = mid_x;
        } else if (obs->seen_pos_x) {
            est_x = obs->hit_min_x + TOF_OBS_HALF_SIZE_CM;
        } else if (obs->seen_neg_x) {
            est_x = obs->hit_max_x - TOF_OBS_HALF_SIZE_CM;
        } else {
            est_x = mid_x;
        }

        if ((obs->seen_pos_y && obs->seen_neg_y) || (span_y >= 8.0f)) {
            est_y = mid_y;
        } else if (obs->seen_pos_y) {
            est_y = obs->hit_min_y + TOF_OBS_HALF_SIZE_CM;
        } else if (obs->seen_neg_y) {
            est_y = obs->hit_max_y - TOF_OBS_HALF_SIZE_CM;
        } else {
            est_y = mid_y;
        }
    } else if (obs->has_manual_prior) {
        /* Giu tam manual prior (chi kep vao mien kha thi cua hit) cho den khi quet du >= 3.5cm mat coc */
        est_x = obs->base_x_cm;
        est_y = obs->base_y_cm;
    } else {
        est_x = mid_x + TOF_OBS_HALF_SIZE_CM * avg_dx;
        est_y = mid_y + TOF_OBS_HALF_SIZE_CM * avg_dy;
    }

    clamp_to_feasible_bounds(obs, &est_x, &est_y);
    obs->base_x_cm = est_x;
    obs->base_y_cm = est_y;
}

static bool ray_intersects_box(float sx, float sy, float dx, float dy, float d_free,
                               float cx, float cy, float *out_t_enter, float *out_t_exit)
{
    float x0 = cx - TOF_OBS_HALF_SIZE_CM;
    float x1 = cx + TOF_OBS_HALF_SIZE_CM;
    float y0 = cy - TOF_OBS_HALF_SIZE_CM;
    float y1 = cy + TOF_OBS_HALF_SIZE_CM;

    float t_enter = 0.0f;
    float t_exit  = 1e6f;

    if (fabsf(dx) < 1e-6f) {
        if (sx < x0 || sx > x1) {
            return false;
        }
    } else {
        float inv = 1.0f / dx;
        float t1  = (x0 - sx) * inv;
        float t2  = (x1 - sx) * inv;
        if (t1 > t2) {
            float tmp = t1;
            t1 = t2;
            t2 = tmp;
        }
        if (t1 > t_enter) t_enter = t1;
        if (t2 < t_exit)  t_exit  = t2;
        if (t_enter > t_exit) {
            return false;
        }
    }

    if (fabsf(dy) < 1e-6f) {
        if (sy < y0 || sy > y1) {
            return false;
        }
    } else {
        float inv = 1.0f / dy;
        float t1  = (y0 - sy) * inv;
        float t2  = (y1 - sy) * inv;
        if (t1 > t2) {
            float tmp = t1;
            t1 = t2;
            t2 = tmp;
        }
        if (t1 > t_enter) t_enter = t1;
        if (t2 < t_exit)  t_exit  = t2;
        if (t_enter > t_exit) {
            return false;
        }
    }

    if (out_t_enter != NULL) *out_t_enter = t_enter;
    if (out_t_exit  != NULL) *out_t_exit  = t_exit;

    return (t_exit > 0.05f) && (t_enter < d_free - 0.05f) && (t_exit > t_enter + 0.05f);
}

static bool apply_carve_ray(const DynamicObstacle_t *obs, const CarveRay_t *ray,
                            float scan_max_cm, uint8_t check_prev_count,
                            float *io_cx, float *io_cy)
{
    float cx = *io_cx;
    float cy = *io_cy;

    if (ray->is_oor) {
        if (dist_point_to_box_cm(ray->sx, ray->sy, cx, cy) < OBS_CARVE_OOR_MIN_DIST_CM) {
            return false;
        }
    }

    float t_enter = 0.0f;
    float t_exit  = 0.0f;
    if (!ray_intersects_box(ray->sx, ray->sy, ray->dx, ray->dy, ray->d_free,
                            cx, cy, &t_enter, &t_exit)) {
        return false;
    }

    bool allow_perp = ray->is_oor || (ray->d_meas > scan_max_cm) || (ray->d_free >= t_exit - 0.5f);

    float cands_x[3];
    float cands_y[3];
    int   cand_cnt = 0;

    if (allow_perp) {
        float nx = -ray->dy;
        float ny =  ray->dx;
        float r_box    = TOF_OBS_HALF_SIZE_CM * (fabsf(nx) + fabsf(ny));
        float signed_d = (cx - ray->sx) * nx + (cy - ray->sy) * ny;
        float shift_p  = r_box - signed_d + 0.15f;
        float shift_m  = r_box + signed_d + 0.15f;
        if (shift_p > 0.0f) {
            cands_x[cand_cnt] = cx + shift_p * nx;
            cands_y[cand_cnt] = cy + shift_p * ny;
            cand_cnt++;
        }
        if (shift_m > 0.0f) {
            cands_x[cand_cnt] = cx - shift_m * nx;
            cands_y[cand_cnt] = cy - shift_m * ny;
            cand_cnt++;
        }
    }

    if (!allow_perp || (t_exit > ray->d_free + 0.5f)) {
        float fwd = (ray->d_free - t_enter) + 0.15f;
        if (fwd > 0.0f) {
            cands_x[cand_cnt] = cx + fwd * ray->dx;
            cands_y[cand_cnt] = cy + fwd * ray->dy;
            cand_cnt++;
        }
    }

    bool  found_clear = false;
    float best_cx = cx;
    float best_cy = cy;
    float best_cost2 = 1e9f;

    bool  found_partial = false;
    float part_cx = cx;
    float part_cy = cy;
    float best_t_enter = t_enter;

    for (int c = 0; c < cand_cnt; c++) {
        float tx = cands_x[c];
        float ty = cands_y[c];
        clamp_to_feasible_bounds(obs, &tx, &ty);

        float bdx = tx - obs->base_x_cm;
        float bdy = ty - obs->base_y_cm;
        float bdist = sqrtf(bdx * bdx + bdy * bdy);
        if (bdist > OBS_MAX_CARVE_SHIFT_CM && bdist > 1e-4f) {
            float scale = OBS_MAX_CARVE_SHIFT_CM / bdist;
            tx = obs->base_x_cm + bdx * scale;
            ty = obs->base_y_cm + bdy * scale;
            clamp_to_feasible_bounds(obs, &tx, &ty);
        }

        if (Map_DistToStaticObstacleCm(tx, ty) <= OBS_STATIC_CENTER_TOL_CM) {
            continue;
        }

        bool breaks_prev = false;
        for (uint8_t p = 0U; p < check_prev_count; p++) {
            const CarveRay_t *pr = &obs->carve_rays[p];
            if (pr->is_oor && dist_point_to_box_cm(pr->sx, pr->sy, tx, ty) < OBS_CARVE_OOR_MIN_DIST_CM) {
                continue;
            }
            bool was_hit = ray_intersects_box(pr->sx, pr->sy, pr->dx, pr->dy, pr->d_free,
                                              cx, cy, NULL, NULL);
            bool now_hit = ray_intersects_box(pr->sx, pr->sy, pr->dx, pr->dy, pr->d_free,
                                              tx, ty, NULL, NULL);
            if (!was_hit && now_hit) {
                breaks_prev = true;
                break;
            }
        }
        if (breaks_prev) {
            continue;
        }

        float nt_enter = 0.0f;
        float nt_exit  = 0.0f;
        bool still_hits = ray_intersects_box(ray->sx, ray->sy, ray->dx, ray->dy, ray->d_free,
                                             tx, ty, &nt_enter, &nt_exit);
        float mdx = tx - cx;
        float mdy = ty - cy;
        float move2 = mdx * mdx + mdy * mdy;
        if (move2 < 1e-6f) {
            continue;
        }

        if (!still_hits) {
            if (!found_clear || move2 < best_cost2) {
                found_clear = true;
                best_cost2  = move2;
                best_cx     = tx;
                best_cy     = ty;
            }
        } else if (!found_clear && (nt_enter > best_t_enter + 0.05f)) {
            found_partial = true;
            best_t_enter  = nt_enter;
            part_cx       = tx;
            part_cy       = ty;
        }
    }

    if (found_clear) {
        *io_cx = best_cx;
        *io_cy = best_cy;
        return true;
    }
    if (found_partial) {
        *io_cx = part_cx;
        *io_cy = part_cy;
        return true;
    }
    return false;
}

static void record_carve_ray(DynamicObstacle_t *obs, const CarveRay_t *new_ray,
                             float scan_max_cm, float cur_x, float cur_y)
{
    if (obs->carve_ray_count < OBS_MAX_CARVE_RAYS) {
        obs->carve_rays[obs->carve_ray_count++] = *new_ray;
        return;
    }

    int   best_slot = -1;
    float best_dot  = 0.90f;
    for (uint8_t m = 0U; m < OBS_MAX_CARVE_RAYS; m++) {
        float dot = new_ray->dx * obs->carve_rays[m].dx +
                    new_ray->dy * obs->carve_rays[m].dy;
        if (dot > best_dot) {
            best_dot  = dot;
            best_slot = (int)m;
        }
    }
    if (best_slot < 0) {
        return;
    }

    /* Chi thay the neu tap rang buoc moi khong lam tut lui vi tri da carve */
    float test_x = obs->base_x_cm;
    float test_y = obs->base_y_cm;
    for (uint8_t m = 0U; m < OBS_MAX_CARVE_RAYS; m++) {
        const CarveRay_t *r = (m == (uint8_t)best_slot) ? new_ray : &obs->carve_rays[m];
        (void)apply_carve_ray(obs, r, scan_max_cm, m, &test_x, &test_y);
    }
    float ddx = test_x - cur_x;
    float ddy = test_y - cur_y;
    if ((ddx * ddx + ddy * ddy) <= (0.05f * 0.05f)) {
        obs->carve_rays[best_slot] = *new_ray;
    }
}

static int8_t find_matching_obstacle_for_hit(const ObstacleMapper_t *mapper, float hx, float hy,
                                             const CarveRay_t *ray)
{
    int8_t best_idx = -1;
    float  best_d2  = 1e9f;
    uint8_t cnt = (mapper->count <= OBS_MAX_DYNAMIC) ? mapper->count : (uint8_t)OBS_MAX_DYNAMIC;

    for (uint8_t k = 0U; k < cnt; k++) {
        const DynamicObstacle_t *obs = &mapper->obstacles[k];
        if (!obs->active) {
            continue;
        }

        if (ray != NULL) {
            float te = 0.0f;
            float tx = 0.0f;
            if (ray_intersects_box(ray->sx, ray->sy, ray->dx, ray->dy, ray->d_free,
                                   obs->center_x_cm, obs->center_y_cm, &te, &tx) &&
                (ray->d_free >= tx - 0.5f)) {
                /* Tia da di xuyen qua toan bo box cua obs[k]: day la tia carve, khong phai hit vao obs[k] */
                continue;
            }
        }

        float dx = hx - obs->center_x_cm;
        float dy = hy - obs->center_y_cm;
        float d2 = dx * dx + dy * dy;

        if (obs->has_hit_bbox) {
            float span_x = fmaxf(obs->hit_max_x, hx) - fminf(obs->hit_min_x, hx);
            float span_y = fmaxf(obs->hit_max_y, hy) - fminf(obs->hit_min_y, hy);
            if (span_x <= 12.5f && span_y <= 12.5f && d2 <= (22.0f * 22.0f)) {
                if (d2 < best_d2) {
                    best_d2  = d2;
                    best_idx = (int8_t)k;
                }
            }
        } else {
            float d_box = dist_point_to_box_cm(hx, hy, obs->center_x_cm, obs->center_y_cm);
            if ((d2 <= (OBS_REFINE_RADIUS_CM * OBS_REFINE_RADIUS_CM)) || (d_box <= 4.0f)) {
                if (d2 < best_d2) {
                    best_d2  = d2;
                    best_idx = (int8_t)k;
                }
            }
        }
    }
    return best_idx;
}

static int8_t find_nearest_registered(const ObstacleMapper_t *mapper, float cx, float cy)
{
    int8_t best_idx = -1;
    float  best_d2  = OBS_DEDUP_RADIUS_CM * OBS_DEDUP_RADIUS_CM;
    uint8_t cnt = (mapper->count <= OBS_MAX_DYNAMIC) ? mapper->count : (uint8_t)OBS_MAX_DYNAMIC;

    for (uint8_t i = 0U; i < cnt; i++) {
        if (!mapper->obstacles[i].active) {
            continue;
        }
        float dx = cx - mapper->obstacles[i].center_x_cm;
        float dy = cy - mapper->obstacles[i].center_y_cm;
        float d2 = dx * dx + dy * dy;
        if (d2 <= best_d2) {
            best_d2  = d2;
            best_idx = (int8_t)i;
        }
    }
    return best_idx;
}

void ObstacleMapper_Init(ObstacleMapper_t *mapper)
{
    if (mapper == NULL) {
        return;
    }
    memset(mapper, 0, sizeof(*mapper));
    mapper->min_clearance_cm = 999.0f;
}

static bool is_already_registered(const ObstacleMapper_t *mapper, float cx, float cy)
{
    return (find_nearest_registered(mapper, cx, cy) >= 0);
}

bool ObstacleMapper_RegisterManual(ObstacleMapper_t *mapper, float cx_cm, float cy_cm)
{
    if (mapper == NULL || !isfinite(cx_cm) || !isfinite(cy_cm)) {
        return false;
    }
    /* Cho phep coc dat sat vach dich (y=5cm, vd MODE_E 145,5) hoac sat tuong (x=175cm) */
    if (cx_cm < 2.0f || cx_cm > 178.0f || cy_cm < 2.0f || cy_cm > 118.0f) {
        return false;
    }
    clamp_arena_center(&cx_cm, &cy_cm);

    if (is_already_registered(mapper, cx_cm, cy_cm)) {
        return false;
    }
    if (mapper->count >= OBS_MAX_DYNAMIC) {
        return false;
    }

    DynamicObstacle_t *obs = &mapper->obstacles[mapper->count];
    memset(obs, 0, sizeof(*obs));
    obs->center_x_cm    = cx_cm;
    obs->center_y_cm    = cy_cm;
    obs->base_x_cm      = cx_cm;
    obs->base_y_cm      = cy_cm;
    obs->committed_x_cm = cx_cm;
    obs->committed_y_cm = cy_cm;
    obs->active           = true;
    obs->has_hit_bbox     = false;
    obs->has_manual_prior = true;
    mapper->count++;
    return true;
}

static uint8_t create_obstacle_from_clusters(ObstacleMapper_t *mapper,
                                             const RayCluster_t *c1,
                                             const RayCluster_t *c2)
{
    uint8_t idx = mapper->count;
    DynamicObstacle_t *obs = &mapper->obstacles[idx];
    memset(obs, 0, sizeof(*obs));
    obs->active = true;

    if (c1 != NULL) {
        for (uint8_t k = 0U; k < c1->streak; k++) {
            obs_add_hit(obs, c1->hit_x[k], c1->hit_y[k], c1->dir_x[k], c1->dir_y[k]);
        }
    }
    if (c2 != NULL) {
        for (uint8_t k = 0U; k < c2->streak; k++) {
            obs_add_hit(obs, c2->hit_x[k], c2->hit_y[k], c2->dir_x[k], c2->dir_y[k]);
        }
    }

    estimate_center_from_hits(obs);
    obs->center_x_cm = obs->base_x_cm;
    obs->center_y_cm = obs->base_y_cm;
    mapper->count++;
    return idx;
}

ObstacleScanResult_t ObstacleMapper_Update(ObstacleMapper_t *mapper,
                                           const RobotPose_t *pose,
                                           const RobotSensorFrame_t *sensors,
                                           bool scan_dynamic_enabled,
                                           bool restrict_to_o3_o5)
{
    ObstacleScanResult_t res;
    memset(&res, 0, sizeof(res));
    res.min_clearance_cm = 999.0f;

    if (mapper == NULL || pose == NULL || sensors == NULL) {
        return res;
    }

    const uint16_t dists_mm[3] = {
        sensors->tof_left_mm,
        sensors->tof_front_mm,
        sensors->tof_right_mm
    };
    const bool valids[3] = {
        sensors->tof_left_valid,
        sensors->tof_front_valid,
        sensors->tof_right_valid
    };

    /* 1. Tinh cu ly an toan nho nhat & co phanh khan cap phia truoc */
    for (int i = 0; i < 3; i++) {
        if (valids[i] && dists_mm[i] >= TOF_MIN_VALID_MM && dists_mm[i] < TOF_OUT_OF_RANGE_MM) {
            float d_cm = (float)dists_mm[i] * 0.1f;
            if (d_cm < res.min_clearance_cm) {
                res.min_clearance_cm = d_cm;
            }
        }
    }

    res.front_emergency = valids[1] &&
                          (dists_mm[1] >= TOF_MIN_VALID_MM) &&
                          (dists_mm[1] <= TOF_EMERGENCY_GUARD_MM);

    mapper->min_clearance_cm = res.min_clearance_cm;
    mapper->front_emergency  = res.front_emergency;

    if (!scan_dynamic_enabled) {
        for (int i = 0; i < 3; i++) {
            clear_ray_cluster(&mapper->rays[i]);
            mapper->oor_streak[i]  = 0U;
            mapper->stale_ticks[i] = 0U;
        }
        return res;
    }

    uint16_t scan_max_mm = (g_cfg.tof_scan_max_mm >= 150U) ? g_cfg.tof_scan_max_mm : (uint16_t)TOF_SCAN_MAX_MM;
    float    scan_max_cm = (float)scan_max_mm * 0.1f;
    float    scan_min_x  = (g_cfg.magic == RUNTIME_CFG_MAGIC) ? g_cfg.scan_trigger_x_cm : OBS_SCAN_ZONE_MIN_X_CM;

    PassFreeRay_t free_rays[3];
    memset(free_rays, 0, sizeof(free_rays));
    for (int i = 0; i < 3; i++) {
        free_rays[i].matched_obs_idx = -1;
    }

    bool obs_dirty[OBS_MAX_DYNAMIC] = { false, false, false, false };
    bool obs_is_new[OBS_MAX_DYNAMIC] = { false, false, false, false };

    /* 2. Pass 1: Chieu tung tia tuoi, cap nhat hit bbox/cluster & thu thap tia free-space */
    for (int i = 0; i < 3; i++) {
        RayCluster_t *rc = &mapper->rays[i];

        if (sensors->tof_is_stale[i]) {
            if (mapper->stale_ticks[i] < 255U) {
                mapper->stale_ticks[i]++;
            }
            if (mapper->stale_ticks[i] >= OBS_STALE_RESET_TICKS) {
                clear_ray_cluster(rc);
                mapper->oor_streak[i] = 0U;
            }
            continue;
        }
        mapper->stale_ticks[i] = 0U;

        float ray_rad = pose->theta_rad + s_ray_angles_rad[i];
        float cos_r   = cosf(ray_rad);
        float sin_r   = sinf(ray_rad);
        float sx      = pose->x_cm + s_ray_offsets_cm[i] * cos_r;
        float sy      = pose->y_cm + s_ray_offsets_cm[i] * sin_r;

        if (valids[i] && dists_mm[i] >= TOF_MIN_VALID_MM && dists_mm[i] < TOF_OUT_OF_RANGE_MM) {
            mapper->oor_streak[i] = 0U;
            float d_meas = (float)dists_mm[i] * 0.1f;
            float d_free = fminf(d_meas - OBS_FREE_RAY_MARGIN_CM, scan_max_cm);
            if (d_free > 0.5f) {
                free_rays[i].has_ray    = true;
                free_rays[i].ray.sx     = sx;
                free_rays[i].ray.sy     = sy;
                free_rays[i].ray.dx     = cos_r;
                free_rays[i].ray.dy     = sin_r;
                free_rays[i].ray.d_meas = d_meas;
                free_rays[i].ray.d_free = d_free;
                free_rays[i].ray.is_oor = false;
            }
        } else if (!valids[i] && dists_mm[i] >= TOF_OUT_OF_RANGE_MM) {
            if (mapper->oor_streak[i] < 255U) {
                mapper->oor_streak[i]++;
            }
            if (mapper->oor_streak[i] >= OBS_CARVE_OOR_MIN_FRAMES) {
                free_rays[i].has_ray    = true;
                free_rays[i].ray.sx     = sx;
                free_rays[i].ray.sy     = sy;
                free_rays[i].ray.dx     = cos_r;
                free_rays[i].ray.dy     = sin_r;
                free_rays[i].ray.d_meas = scan_max_cm + 10.0f;
                free_rays[i].ray.d_free = scan_max_cm;
                free_rays[i].ray.is_oor = true;
            }
        } else {
            mapper->oor_streak[i] = 0U;
        }

        bool in_scan_range = valids[i] &&
                             (dists_mm[i] >= TOF_MIN_VALID_MM) &&
                             (dists_mm[i] <= scan_max_mm);
        if (!in_scan_range) {
            clear_ray_cluster(rc);
            continue;
        }

        float d_meas   = (float)dists_mm[i] * 0.1f;
        float d_hit    = s_ray_offsets_cm[i] + d_meas;
        float d_center = d_hit + TOF_OBS_HALF_SIZE_CM;

        float hx = pose->x_cm + d_hit * cos_r;
        float hy = pose->y_cm + d_hit * sin_r;
        float cx = pose->x_cm + d_center * cos_r;
        float cy = pose->y_cm + d_center * sin_r;

        bool inside_arena = (hx > 1.5f && hx < 178.5f && hy > 0.5f && hy < 118.5f);
        if (!inside_arena) {
            clear_ray_cluster(rc);
            continue;
        }

        clamp_arena_center(&cx, &cy);
        bool in_scan_zone = (!restrict_to_o3_o5) || (cx >= scan_min_x && cy >= 2.0f);
        bool near_static  = (Map_DistToStaticObstacleCm(hx, hy) <= OBS_STATIC_HIT_TOL_CM) ||
                            (Map_DistToStaticObstacleCm(cx, cy) <= OBS_STATIC_CENTER_TOL_CM);
        if (!in_scan_zone || near_static) {
            clear_ray_cluster(rc);
            continue;
        }

        const CarveRay_t *ray_ptr = free_rays[i].has_ray ? &free_rays[i].ray : NULL;
        int8_t match_idx = find_matching_obstacle_for_hit(mapper, hx, hy, ray_ptr);
        if (match_idx >= 0) {
            obs_add_hit(&mapper->obstacles[match_idx], hx, hy, cos_r, sin_r);
            obs_dirty[match_idx] = true;
            free_rays[i].matched_obs_idx = match_idx;
            clear_ray_cluster(rc);
            continue;
        }

        int8_t dedup_idx = find_nearest_registered(mapper, cx, cy);
        if (dedup_idx >= 0) {
            bool passed_through = false;
            if (free_rays[i].has_ray) {
                float te = 0.0f;
                float tx = 0.0f;
                const DynamicObstacle_t *dobs = &mapper->obstacles[dedup_idx];
                if (ray_intersects_box(sx, sy, cos_r, sin_r, free_rays[i].ray.d_free,
                                       dobs->center_x_cm, dobs->center_y_cm, &te, &tx) &&
                    (free_rays[i].ray.d_free >= tx - 0.5f)) {
                    passed_through = true;
                }
            }
            if (!passed_through) {
                free_rays[i].matched_obs_idx = dedup_idx;
            }
            clear_ray_cluster(rc);
            continue;
        }

        push_ray_sample(rc, cx, cy, hx, hy, cos_r, sin_r);
    }

    /* 3. Pass 2a: Xac nhan nhanh da cam bien (2 tia cung nhin thay 1 coc) */
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            RayCluster_t *ri = &mapper->rays[i];
            RayCluster_t *rj = &mapper->rays[j];
            bool dual_ready = ((ri->streak >= OBS_CONFIRM_DUAL_RAY_FRAMES) && (rj->streak >= 1U)) ||
                              ((ri->streak >= 1U) && (rj->streak >= OBS_CONFIRM_DUAL_RAY_FRAMES));
            if (!dual_ready) {
                continue;
            }

            float axi = ri->sum_x / (float)ri->streak;
            float ayi = ri->sum_y / (float)ri->streak;
            float axj = rj->sum_x / (float)rj->streak;
            float ayj = rj->sum_y / (float)rj->streak;
            float ddx = axi - axj;
            float ddy = ayi - ayj;
            float cd2 = ddx * ddx + ddy * ddy;

            bool center_close = (cd2 <= (OBS_DEDUP_RADIUS_CM * OBS_DEDUP_RADIUS_CM));
            bool hits_compat  = false;
            if (!center_close && cd2 <= (14.5f * 14.5f)) {
                float hxi = ri->hit_x[ri->streak - 1U];
                float hyi = ri->hit_y[ri->streak - 1U];
                float hxj = rj->hit_x[rj->streak - 1U];
                float hyj = rj->hit_y[rj->streak - 1U];
                hits_compat = (fabsf(hxi - hxj) <= 10.5f) && (fabsf(hyi - hyj) <= 10.5f);
            }
            if (!center_close && !hits_compat) {
                continue;
            }

            float avg_x = 0.5f * (axi + axj);
            float avg_y = 0.5f * (ayi + ayj);
            clamp_arena_center(&avg_x, &avg_y);

            if (is_already_registered(mapper, avg_x, avg_y) || mapper->count >= OBS_MAX_DYNAMIC) {
                continue;
            }

            uint8_t new_idx = create_obstacle_from_clusters(mapper, ri, rj);
            obs_dirty[new_idx]  = true;
            obs_is_new[new_idx] = true;
            if (free_rays[i].has_ray) free_rays[i].matched_obs_idx = (int8_t)new_idx;
            if (free_rays[j].has_ray) free_rays[j].matched_obs_idx = (int8_t)new_idx;
            clear_ray_cluster(ri);
            clear_ray_cluster(rj);

            /* Neu tia thu 3 cung nhin vao chinh coc vua tao, gop luon hit va xoa streak */
            int k = 3 - i - j;
            RayCluster_t *rk = &mapper->rays[k];
            if (rk->streak > 0U) {
                uint8_t last_s = (uint8_t)(rk->streak - 1U);
                int8_t m = find_matching_obstacle_for_hit(mapper, rk->hit_x[last_s], rk->hit_y[last_s], NULL);
                float axk = rk->sum_x / (float)rk->streak;
                float ayk = rk->sum_y / (float)rk->streak;
                if (m == (int8_t)new_idx) {
                    for (uint8_t s = 0U; s < rk->streak; s++) {
                        obs_add_hit(&mapper->obstacles[new_idx],
                                    rk->hit_x[s], rk->hit_y[s], rk->dir_x[s], rk->dir_y[s]);
                    }
                    if (free_rays[k].has_ray) free_rays[k].matched_obs_idx = (int8_t)new_idx;
                    clear_ray_cluster(rk);
                } else if (find_nearest_registered(mapper, axk, ayk) == (int8_t)new_idx) {
                    if (free_rays[k].has_ray) free_rays[k].matched_obs_idx = (int8_t)new_idx;
                    clear_ray_cluster(rk);
                }
            }
        }
    }

    /* 4. Pass 2b: Xac nhan don tia (3 khung lien tiep) */
    for (int i = 0; i < 3; i++) {
        RayCluster_t *rc = &mapper->rays[i];
        if (rc->streak < OBS_CONFIRM_FRAMES) {
            continue;
        }

        uint8_t last_s = (uint8_t)(rc->streak - 1U);
        int8_t match_idx = find_matching_obstacle_for_hit(mapper, rc->hit_x[last_s], rc->hit_y[last_s], NULL);
        if (match_idx >= 0) {
            for (uint8_t s = 0U; s < rc->streak; s++) {
                obs_add_hit(&mapper->obstacles[match_idx],
                            rc->hit_x[s], rc->hit_y[s], rc->dir_x[s], rc->dir_y[s]);
            }
            obs_dirty[match_idx] = true;
            if (free_rays[i].has_ray) free_rays[i].matched_obs_idx = match_idx;
            clear_ray_cluster(rc);
            continue;
        }

        float avg_x = rc->sum_x / (float)rc->streak;
        float avg_y = rc->sum_y / (float)rc->streak;
        clamp_arena_center(&avg_x, &avg_y);

        int8_t dedup_idx = find_nearest_registered(mapper, avg_x, avg_y);
        if (dedup_idx >= 0) {
            if (free_rays[i].has_ray) free_rays[i].matched_obs_idx = dedup_idx;
            clear_ray_cluster(rc);
            continue;
        }

        if (mapper->count >= OBS_MAX_DYNAMIC) {
            clear_ray_cluster(rc);
            continue;
        }

        uint8_t new_idx = create_obstacle_from_clusters(mapper, rc, NULL);
        obs_dirty[new_idx]  = true;
        obs_is_new[new_idx] = true;
        if (free_rays[i].has_ray) free_rays[i].matched_obs_idx = (int8_t)new_idx;
        clear_ray_cluster(rc);
    }

    /* 5. Pass 3: Uoc luong tam hinh hoc + ap dung free-space carving ben vung */
    bool any_free_ray = free_rays[0].has_ray || free_rays[1].has_ray || free_rays[2].has_ray;
    uint8_t cnt = (mapper->count <= OBS_MAX_DYNAMIC) ? mapper->count : (uint8_t)OBS_MAX_DYNAMIC;

    for (uint8_t k = 0U; k < cnt; k++) {
        DynamicObstacle_t *obs = &mapper->obstacles[k];
        if (!obs->active) {
            continue;
        }

        if (obs_dirty[k]) {
            estimate_center_from_hits(obs);
        }

        if (obs_dirty[k] || any_free_ray) {
            float cx = obs->base_x_cm;
            float cy = obs->base_y_cm;

            for (uint8_t m = 0U; m < obs->carve_ray_count; m++) {
                (void)apply_carve_ray(obs, &obs->carve_rays[m], scan_max_cm, m, &cx, &cy);
            }

            for (int i = 0; i < 3; i++) {
                if (!free_rays[i].has_ray || free_rays[i].matched_obs_idx == (int8_t)k) {
                    continue;
                }
                float prev_x = cx;
                float prev_y = cy;
                if (apply_carve_ray(obs, &free_rays[i].ray, scan_max_cm,
                                    obs->carve_ray_count, &cx, &cy)) {
                    float mdx = cx - prev_x;
                    float mdy = cy - prev_y;
                    if ((mdx * mdx + mdy * mdy) >= (0.05f * 0.05f)) {
                        record_carve_ray(obs, &free_rays[i].ray, scan_max_cm, cx, cy);
                    }
                }
            }

            obs->center_x_cm = cx;
            obs->center_y_cm = cy;
            obs->carve_dx_cm = cx - obs->base_x_cm;
            obs->carve_dy_cm = cy - obs->base_y_cm;
        }

        if (obs_is_new[k]) {
            obs->committed_x_cm = obs->center_x_cm;
            obs->committed_y_cm = obs->center_y_cm;
            if (!res.new_obstacle_found) {
                res.new_obstacle_found = true;
                res.new_obs_x_cm       = obs->center_x_cm;
                res.new_obs_y_cm       = obs->center_y_cm;
            }
        } else {
            float sdx = obs->center_x_cm - obs->committed_x_cm;
            float sdy = obs->center_y_cm - obs->committed_y_cm;
            if ((sdx * sdx + sdy * sdy) >= (OBS_REFINE_MIN_SHIFT_CM * OBS_REFINE_MIN_SHIFT_CM)) {
                obs->committed_x_cm  = obs->center_x_cm;
                obs->committed_y_cm  = obs->center_y_cm;
                res.obstacle_refined = true;
            }
        }
    }

    return res;
}

uint8_t ObstacleMapper_GetCount(const ObstacleMapper_t *mapper)
{
    return (mapper != NULL) ? mapper->count : 0U;
}

const DynamicObstacle_t *ObstacleMapper_GetObstacle(const ObstacleMapper_t *mapper, uint8_t idx)
{
    if (mapper == NULL || idx >= mapper->count) {
        return NULL;
    }
    return &mapper->obstacles[idx];
}
