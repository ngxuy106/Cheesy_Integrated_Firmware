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
    for (uint8_t i = 0U; i < mapper->count; i++) {
        if (!mapper->obstacles[i].active) {
            continue;
        }
        float dx = cx - mapper->obstacles[i].center_x_cm;
        float dy = cy - mapper->obstacles[i].center_y_cm;
        if ((dx * dx + dy * dy) <= (OBS_DEDUP_RADIUS_CM * OBS_DEDUP_RADIUS_CM)) {
            return true;
        }
    }
    return false;
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
    if (cx_cm < 5.0f)   cx_cm = 5.0f;
    if (cx_cm > 175.0f) cx_cm = 175.0f;
    if (cy_cm < 5.0f)   cy_cm = 5.0f;
    if (cy_cm > 115.0f) cy_cm = 115.0f;

    if (is_already_registered(mapper, cx_cm, cy_cm)) {
        return false;
    }
    if (mapper->count >= OBS_MAX_DYNAMIC) {
        return false;
    }
    mapper->obstacles[mapper->count].center_x_cm = cx_cm;
    mapper->obstacles[mapper->count].center_y_cm = cy_cm;
    mapper->obstacles[mapper->count].active      = true;
    mapper->count++;
    return true;
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
            mapper->rays[i].streak = 0U;
        }
        return res;
    }

    /* 2. Chieu tung tia ToF sang he toa do sa ban & loc vat can tinh */
    for (int i = 0; i < 3; i++) {
        RayCluster_t *rc = &mapper->rays[i];

        /* Bo qua mau cu (chua co phep do moi tu VL53L0X) de tranh 1 mau nhieu
         * duoc dem lap thanh 3 khung lien tiep (debounce gia). */
        if (sensors->tof_is_stale[i]) {
            continue;
        }

        uint16_t scan_max_mm = (g_cfg.tof_scan_max_mm >= 150U) ? g_cfg.tof_scan_max_mm : (uint16_t)TOF_SCAN_MAX_MM;
        bool in_scan_range = valids[i] &&
                             (dists_mm[i] >= TOF_MIN_VALID_MM) &&
                             (dists_mm[i] <= scan_max_mm);

        if (!in_scan_range) {
            rc->streak = 0U;
            continue;
        }

        float ray_rad  = pose->theta_rad + s_ray_angles_rad[i];
        float cos_r    = cosf(ray_rad);
        float sin_r    = sinf(ray_rad);
        float d_meas   = (float)dists_mm[i] * 0.1f;
        float d_hit    = s_ray_offsets_cm[i] + d_meas;
        float d_center = d_hit + TOF_OBS_HALF_SIZE_CM;

        /* Diem tia cham mat ngoai vat can */
        float hx = pose->x_cm + d_hit * cos_r;
        float hy = pose->y_cm + d_hit * sin_r;

        /* Uoc luong tam khoi xop 10x10cm; neu tia cheo ban vao coc dat sat
         * vach dich (y=0) hoac sat tuong, kep tam ve [5..175] x [5..115] */
        float cx = pose->x_cm + d_center * cos_r;
        float cy = pose->y_cm + d_center * sin_r;

        /* Loai bo diem cham nam ngoai sa ban hoac ngoai vung O4..O5 (neu bat restrict).
         * Cho phep hy > 0.5cm de bat duoc coc dat sat vach dich nhu MODE_E (145, 5). */
        bool inside_arena = (hx > 1.5f && hx < 178.5f && hy > 0.5f && hy < 118.5f);
        if (!inside_arena) {
            rc->streak = 0U;
            continue;
        }

        if (cx < 5.0f)   cx = 5.0f;
        if (cx > 175.0f) cx = 175.0f;
        if (cy < 5.0f)   cy = 5.0f;
        if (cy > 115.0f) cy = 115.0f;

        float scan_min_x = (g_cfg.magic == RUNTIME_CFG_MAGIC) ? g_cfg.scan_trigger_x_cm : OBS_SCAN_ZONE_MIN_X_CM;
        bool in_scan_zone = (!restrict_to_o3_o5) || (cx >= scan_min_x && cy >= 2.0f);

        /* Kiem tra kep vat can tinh:
         * - Tia trung coc tinh/tuong se co diem cham (hx,hy) <= 2.6cm hoac tam (cx,cy) <= 3.2cm.
         * - Coc dong dat ke sat coc tinh/chu U (vd MODE_C sat chu U, MODE_E duoi T16)
         *   van co tam uoc luong cach vat tinh ~5cm (> 3.2cm) -> KHONG bi loc nham! */
        bool near_static = (Map_DistToStaticObstacleCm(hx, hy) <= OBS_STATIC_HIT_TOL_CM) ||
                           (Map_DistToStaticObstacleCm(cx, cy) <= OBS_STATIC_CENTER_TOL_CM);

        if (!in_scan_zone || near_static || is_already_registered(mapper, cx, cy)) {
            rc->streak = 0U;
            continue;
        }

        /* 3. Gom cum khong gian 3 khung lien tiep de triet nhieu tia laser */
        if (rc->streak == 0U) {
            rc->sum_x  = cx;
            rc->sum_y  = cy;
            rc->last_x = cx;
            rc->last_y = cy;
            rc->streak = 1U;
        } else {
            float dx = cx - rc->last_x;
            float dy = cy - rc->last_y;
            if ((dx * dx + dy * dy) <= (OBS_CLUSTER_RADIUS_CM * OBS_CLUSTER_RADIUS_CM)) {
                rc->sum_x += cx;
                rc->sum_y += cy;
                rc->last_x = cx;
                rc->last_y = cy;
                rc->streak++;
            } else {
                rc->sum_x  = cx;
                rc->sum_y  = cy;
                rc->last_x = cx;
                rc->last_y = cy;
                rc->streak = 1U;
            }
        }

        if (rc->streak >= OBS_CONFIRM_FRAMES && !res.new_obstacle_found) {
            float avg_x = rc->sum_x / (float)rc->streak;
            float avg_y = rc->sum_y / (float)rc->streak;
            rc->streak = 0U;

            if (ObstacleMapper_RegisterManual(mapper, avg_x, avg_y)) {
                res.new_obstacle_found = true;
                res.new_obs_x_cm       = avg_x;
                res.new_obs_y_cm       = avg_y;
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
