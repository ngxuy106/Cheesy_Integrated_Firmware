/**
 * ============================================================================
 * @file    obstacle_mapper.h
 * @brief   Bo quet map & phat hien vat can dong tu hanh bang 3x VL53L0X
 *          (Trai +45 deg, Truoc 0 deg, Phai -45 deg)
 * ============================================================================
 */

#ifndef OBSTACLE_MAPPER_H
#define OBSTACLE_MAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_interface.h"
#include "robot_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float sx;
    float sy;
    float dx;
    float dy;
    float d_meas;
    float d_free;
    bool  is_oor;
} CarveRay_t;

typedef struct {
    float      center_x_cm;
    float      center_y_cm;
    bool       active;
    bool       has_hit_bbox;
    bool       has_manual_prior;
    float      hit_min_x;
    float      hit_max_x;
    float      hit_min_y;
    float      hit_max_y;
    float      sum_dx;
    float      sum_dy;
    bool       seen_pos_x;
    bool       seen_neg_x;
    bool       seen_pos_y;
    bool       seen_neg_y;
    float      base_x_cm;
    float      base_y_cm;
    float      committed_x_cm;
    float      committed_y_cm;
    float      carve_dx_cm;
    float      carve_dy_cm;
    CarveRay_t carve_rays[OBS_MAX_CARVE_RAYS];
    uint8_t    carve_ray_count;
} DynamicObstacle_t;

typedef struct {
    float   sum_x;
    float   sum_y;
    float   last_x;
    float   last_y;
    float   cx[OBS_CONFIRM_FRAMES];
    float   cy[OBS_CONFIRM_FRAMES];
    float   hit_x[OBS_CONFIRM_FRAMES];
    float   hit_y[OBS_CONFIRM_FRAMES];
    float   dir_x[OBS_CONFIRM_FRAMES];
    float   dir_y[OBS_CONFIRM_FRAMES];
    uint8_t streak;
} RayCluster_t;

typedef struct {
    DynamicObstacle_t obstacles[OBS_MAX_DYNAMIC];
    uint8_t           count;
    RayCluster_t      rays[3];          /* 0: Left, 1: Front, 2: Right */
    uint8_t           oor_streak[3];    /* Dem so khung OOR tuoi lien tiep */
    uint8_t           stale_ticks[3];   /* Dem so khung stale lien tiep   */
    float             min_clearance_cm; /* Khoang cach gan nhat trong 3 tia */
    bool              front_emergency;  /* true neu phia truoc < 8.5cm */
} ObstacleMapper_t;

typedef struct {
    bool  new_obstacle_found;
    bool  obstacle_refined;
    float new_obs_x_cm;
    float new_obs_y_cm;
    float min_clearance_cm;
    bool  front_emergency;
} ObstacleScanResult_t;

/* Khoi tao / Xoa toan bo vat can dong & bo loc nhieu */
void ObstacleMapper_Init(ObstacleMapper_t *mapper);

/* Quet 3 tia VL53L0X o chu ky 20ms:
 * - scan_dynamic_enabled: true de kich hoat phat hien & nap vat can la
 * - restrict_to_o3_o5:    true de chi nhan vat can la trong vung O 3..O 5
 *                         (cx >= OBS_SCAN_ZONE_MIN_X_CM = 60 cm, chong bao gia O 1..O 2) */
ObstacleScanResult_t ObstacleMapper_Update(ObstacleMapper_t *mapper,
                                           const RobotPose_t *pose,
                                           const RobotSensorFrame_t *sensors,
                                           bool scan_dynamic_enabled,
                                           bool restrict_to_o3_o5);

/* Nap thu cong 1 vat can dong (vd tu lenh BLE OBS hoac recovery backup) */
bool ObstacleMapper_RegisterManual(ObstacleMapper_t *mapper, float cx_cm, float cy_cm);

/* Kiem tra so luong & lay danh sach vat can dong da ghi nhan */
uint8_t ObstacleMapper_GetCount(const ObstacleMapper_t *mapper);
const DynamicObstacle_t *ObstacleMapper_GetObstacle(const ObstacleMapper_t *mapper, uint8_t idx);

#ifdef __cplusplus
}
#endif

#endif /* OBSTACLE_MAPPER_H */
