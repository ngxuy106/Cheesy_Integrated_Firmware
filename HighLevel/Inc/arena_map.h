/**
 * ============================================================================
 * @file    arena_map.h
 * @brief   Ban do C-Space 50x74 (2.5cm/o) sa ban thi dau 180x120cm (17 coc tinh)
 * ============================================================================
 * Ke thua 100% ban do da xac minh tu Firmware_Workspace/arena_map.h + bo sung
 * ham hinh hoc tinh khoang cach chinh xac toi vat can tinh cho cam bien ToF.
 * ============================================================================
 */

#ifndef ARENA_MAP_H
#define ARENA_MAP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Kich thuoc ma tran C-Space sa ban */
#define MAP_CELL_MM             25
#define MAP_NX                  74      /* Cot X: 0..73 (x = -2.5 .. 182.5 cm) */
#define MAP_NY                  50      /* Hang Y: 0..49 (y = -2.5 .. 122.5 cm)*/
#define MAP_ROWS                MAP_NY
#define MAP_COLS                MAP_NX
#define MAP_MARGIN              1
#define MAP_INFLATION_CELLS     3
#define GRID_SIZE_CM            2.5f
#define MAP_Y_OFFSET_CM         (-2.5f)
#define NUM_STD_OBSTACLES       17

/* Toa do xuat phat O 1 (CO DINH sau vach xuat phat, huong +Y = 90 deg) */
#define ROBOT_START_X           30.0f
#define ROBOT_START_Y           (-6.0f)
#define ROBOT_START_YAW         90.0f

/* Vung ve dich O 5 (Ca xe 10cm vuot qua vach Y = 0 ke ca khi odom lech 2cm) */
#define GOAL_LINE_Y             0.0f
#define GOAL_FINISH_Y           (-7.0f)
#define GOAL_X_MIN              128.0f
#define GOAL_X_MAX              172.0f
#define ROBOT_GOAL_X            150.0f
#define ROBOT_GOAL_Y            (-8.0f)

/* Ma loai o trong luoi C-Space */
#define CELL_FREE               0U
#define CELL_OBST               1U
#define CELL_BLOCK              2U
#define CELL_WALL               3U
#define CELL_INFLATED           4U
#define CELL_OBSTACLE           1U

typedef struct {
    const char *id;
    int32_t     center_x_mm;
    int32_t     center_y_mm;
    const char *zone_desc;
} Obstacle_t;

extern const Obstacle_t OBSTACLES_STD[NUM_STD_OBSTACLES];

/* Quy doi toa do <-> chi so luoi */
int     Map_GetGridCol(float x_cm);
int     Map_GetGridRow(float y_cm);
float   Map_GetWorldX(int col);
float   Map_GetWorldY(int row);

int     Map_GetColFromMm(int32_t x_mm);
int     Map_GetRowFromMm(int32_t y_mm);
int32_t Map_GetWorldX_Mm(int ix);
int32_t Map_GetWorldY_Mm(int iy);

/* Tra cuu vat can & vach dich */
bool           Map_IsObstacle(int row, int col);
bool           Map_IsBlocked(int32_t x_mm, int32_t y_mm);
bool           Map_IsBlockedCm(float x_cm, float y_cm);
bool           Map_IsFinishLineCrossed(float x_cm, float y_cm);
uint8_t        Map_GetCellType(int row, int col);
const uint8_t *Map_GetGridPtr(void);

/* Khoang cach hinh hoc (cm) tu diem (x_cm, y_cm) den vat can tinh gan nhat
 * (17 coc T1..T17 kich thuoc 10x10cm, khoi chu U [60..120, 0..60], tuong bien).
 * Dung cho Obstacle Mapper loai tru tia ToF trung tuong / coc tinh. */
float Map_DistToStaticObstacleCm(float x_cm, float y_cm);

#ifdef __cplusplus
}
#endif

#endif /* ARENA_MAP_H */
