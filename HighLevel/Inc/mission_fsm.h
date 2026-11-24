/**
 * ============================================================================
 * @file    mission_fsm.h
 * @brief   May trang thai tu hanh hop nhat (Chay gop 1 che do duy nhat:
 *          tu O 3 -> O 5 luon bat 3x VL53L0X quet map & ne vat can bang A*,
 *          toc do mong muon v_target hieu chinh duoc sau moi lan test)
 * ============================================================================
 */

#ifndef MISSION_FSM_H
#define MISSION_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_interface.h"
#include "astar_planner.h"
#include "ekf_localization.h"
#include "obstacle_mapper.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FSM_IDLE = 0,               /* Cho bam nut PA11 hoac BLE RUN:2                  */
    FSM_RUN_O1_O2,              /* Chay tu hanh O 1 -> O 2 (chua bat quet vat dong) */
    FSM_RUN_SCAN_O3_O5,         /* Tu O 3 -> O 5: bat 3x ToF quet map & A* replan   */
    FSM_RECOVERY_BACKUP,        /* Lui an toan 6cm + replan khi sat vat / ket banh  */
    FSM_MISSION_FINISHED,       /* Can dich O 5 thanh cong, dung & chot diem        */
    FSM_EMERGENCY_STOP          /* Dung khan cap (giu PA11 / BLE STOP / pin yeu)    */
} FSMState_t;

typedef enum {
    TRACK_SEG_O1_START_NARROW = 0,
    TRACK_SEG_O2_CORNER,
    TRACK_SEG_O3_STRAIGHT,
    TRACK_SEG_O4_CORNER_SCAN,
    TRACK_SEG_O5_FINISH_GATE,
    TRACK_SEG_COUNT
} TrackSegment_t;

typedef struct {
    float    v_target_cms;      /* Van toc mong muon cho doan (cm/s, hieu chinh duoc) */
    float    min_tof_dist_cm;   /* Cu ly ToF nho nhat do duoc trong lan chay (cm)     */
    float    max_curvature;     /* Do cong lon nhat da chay trong lan chay (1/cm)     */
    uint32_t elapsed_ms;        /* Thoi gian chay qua doan nay (ms)                   */
} SegmentProfile_t;

/* Khoi tao toan bo Tang Cao (EKF, Obstacle Mapper, A*, Pure Pursuit, FSM) */
void Mission_Init(void);

/* Chu ky xu ly chinh 50Hz (20ms):
 * Doc khung cam bien -> cap nhat EKF -> quet ToF -> FSM + A-Star + PurePursuit -> xuat lenh */
void Mission_Step(const RobotSensorFrame_t *sensors, RobotDriveCmd_t *out_cmd);

/* Hieu chinh van toc mong muon v_target (duoc giu nguyen qua cac lan Reset/chay tiep) */
bool  Mission_SetSegmentTargetSpeed(TrackSegment_t seg, float v_target_cms);
void  Mission_SetAllTargetSpeeds(float v_o1, float v_o2, float v_o3, float v_o4, float v_o5);
bool  Mission_SetUniformTargetSpeed(float v_target_cms);
float Mission_GetSegmentTargetSpeed(TrackSegment_t seg);

/* Nap vat can thu cong (ho tro lenh BLE OBS:<zone>,<pos> neu can test) */
bool Mission_InjectObstacle(float obs_x_cm, float obs_y_cm);

/* Dat lai EKF + FSM ve trang thai IDLE (vd lenh BLE ZERO), giu nguyen v_target da chinh */
void Mission_ResetAll(bool imu_valid, float current_imu_yaw_deg);

/* Cac ham truy van trang thai & chan doan (cho Telemetry & Unit Tests) */
FSMState_t              Mission_GetState(void);
RobotPose_t             Mission_GetPose(void);
const EKF_Localizer_t  *Mission_GetEKF(void);
const ObstacleMapper_t *Mission_GetMapper(void);
RobotStatus_t           Mission_GetStatus(void);
TrackSegment_t          Mission_DetectSegment(RobotPose_t pose);
const SegmentProfile_t *Mission_GetSegmentProfile(TrackSegment_t seg);
int                     Mission_GetTotalWaypoints(void);
const Waypoint_t       *Mission_GetPath(void);
int32_t                 Mission_CalculateScore(void);

#ifdef __cplusplus
}
#endif

#endif /* MISSION_FSM_H */
