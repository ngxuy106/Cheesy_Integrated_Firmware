/**
 * ============================================================================
 * @file    runtime_config.h
 * @brief   Cau hinh dong bo tai thoi gian chay (BLE Live Tuning + Luu Flash)
 * ============================================================================
 * - Chinh qua Bluetooth HC-05 (CFG:<KEY>,<VAL>) ma KHONG can nap lai code.
 * - Luu xuong Flash Sector 7 (0x08060000, 128 KB cuoi cua STM32F411CEU6) qua
 *   lenh CFG:SAVE de giu lai khi tat nguon.
 * ============================================================================
 */

#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RUNTIME_CFG_MAGIC       0x43485331U /* "CHS1" */
#define RUNTIME_CFG_VERSION     1U
#define RUNTIME_CFG_FLASH_ADDR  0x08060000U /* STM32F411 Sector 7 (128 KB) */

typedef struct {
    uint32_t magic;
    uint32_t version;
    float    v_target[5];            /* V1..V5: O 1..O 5 target speed (cm/s) [10..30] */
    float    v_detour_max;           /* VDET: detour cap (cm/s) [10..30]              */
    float    v_max;                  /* VMAX: max wheel & linear speed (cm/s) [15..35]*/
    float    accel_max_cms2;         /* ACC: linear slew rate (cm/s^2) [20..150]      */
    float    motor_kp;               /* KP: motor PI Kp [10..600]                     */
    float    motor_ki;               /* KI: motor PI Ki [0..50]                       */
    float    wheel_diam_cm;          /* WD: wheel diameter (cm) [2.5..8.0]            */
    float    track_width_cm;         /* WB: wheel base (cm) [6.0..16.0]               */
    float    encoder_ppr;            /* PPR: 4x ticks/rev [100..10000]                */
    uint16_t tof_scan_max_mm;        /* TOF: 3x VL53L0X scan range (mm) [150..800]    */
    uint8_t  yaw_invert;             /* YAWINV: 0=normal, 1=invert MPU6050 yaw        */
    uint8_t  reserved;
    float    scan_trigger_x_cm;      /* SCANX: X threshold enabling ToF scan [0..120] */
    float    pp_lookahead_offset_cm; /* LD: Pure Pursuit lookahead offset [4.0..12.0] */
    uint32_t crc32;
} RuntimeConfig_t;

extern RuntimeConfig_t g_cfg;

void  RuntimeConfig_LoadDefaults(RuntimeConfig_t *cfg);
bool  RuntimeConfig_LoadFromFlash(RuntimeConfig_t *cfg);
void  RuntimeConfig_Init(void);
bool  RuntimeConfig_SaveToFlash(const RuntimeConfig_t *cfg);
bool  RuntimeConfig_SetParam(const char *key, float value);
float RuntimeConfig_CmPerTick(void);

#ifdef __cplusplus
}
#endif

#endif /* RUNTIME_CONFIG_H */
