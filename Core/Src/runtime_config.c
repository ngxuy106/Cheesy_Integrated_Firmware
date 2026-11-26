/**
 * ============================================================================
 * @file    runtime_config.c
 * @brief   Hien thuc cau hinh live & luu tru ben vung Flash Sector 7 (CRC32)
 * ============================================================================
 */

#include "runtime_config.h"
#include "robot_config.h"
#include "pure_pursuit.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#ifdef STM32F411xE
#include "stm32f4xx_hal.h"
#endif

RuntimeConfig_t g_cfg;

static char ascii_upper(char c)
{
    return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c;
}

static bool key_equals_nocase(const char *a, const char *b)
{
    if (a == NULL || b == NULL) return false;
    while (*a && *b) {
        if (ascii_upper(*a) != ascii_upper(*b)) return false;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static uint32_t calc_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

static uint32_t compute_cfg_crc(const RuntimeConfig_t *cfg)
{
    return calc_crc32((const uint8_t *)cfg, offsetof(RuntimeConfig_t, crc32));
}

static bool is_valid_range(const RuntimeConfig_t *c)
{
    if (c == NULL) return false;
    for (int i = 0; i < 5; i++) {
        if (!isfinite(c->v_target[i]) || c->v_target[i] < 10.0f || c->v_target[i] > 30.0f) return false;
    }
    if (!isfinite(c->v_detour_max) || c->v_detour_max < 10.0f || c->v_detour_max > 30.0f) return false;
    if (!isfinite(c->v_max) || c->v_max < 15.0f || c->v_max > 35.0f) return false;
    if (!isfinite(c->accel_max_cms2) || c->accel_max_cms2 < 20.0f || c->accel_max_cms2 > 150.0f) return false;
    if (!isfinite(c->motor_kp) || c->motor_kp < 10.0f || c->motor_kp > 600.0f) return false;
    if (!isfinite(c->motor_ki) || c->motor_ki < 0.0f || c->motor_ki > 50.0f) return false;
    if (!isfinite(c->wheel_diam_cm) || c->wheel_diam_cm < 2.5f || c->wheel_diam_cm > 8.0f) return false;
    if (!isfinite(c->track_width_cm) || c->track_width_cm < 6.0f || c->track_width_cm > 16.0f) return false;
    if (!isfinite(c->encoder_ppr) || c->encoder_ppr < 100.0f || c->encoder_ppr > 10000.0f) return false;
    if (c->tof_scan_max_mm < 150U || c->tof_scan_max_mm > 800U) return false;
    if (c->yaw_invert > 1U) return false;
    if (!isfinite(c->scan_trigger_x_cm) || c->scan_trigger_x_cm < 0.0f || c->scan_trigger_x_cm > 120.0f) return false;
    if (!isfinite(c->pp_lookahead_offset_cm) || c->pp_lookahead_offset_cm < 4.0f || c->pp_lookahead_offset_cm > 12.0f) return false;
    return true;
}

void RuntimeConfig_LoadDefaults(RuntimeConfig_t *cfg)
{
    if (cfg == NULL) return;
    memset(cfg, 0, sizeof(*cfg));
    cfg->magic                  = RUNTIME_CFG_MAGIC;
    cfg->version                = RUNTIME_CFG_VERSION;
    cfg->v_target[0]            = V_TARGET_O1_CMS;
    cfg->v_target[1]            = V_TARGET_O2_CMS;
    cfg->v_target[2]            = V_TARGET_O3_CMS;
    cfg->v_target[3]            = V_TARGET_O4_CMS;
    cfg->v_target[4]            = V_TARGET_O5_CMS;
    cfg->v_detour_max           = V_TARGET_DETOUR_MAX_CMS;
    cfg->v_max                  = ROBOT_MAX_SPEED_CMS;
    cfg->accel_max_cms2         = ROBOT_ACCEL_MAX_CMS2;
    cfg->motor_kp               = MOTOR_DEFAULT_KP;
    cfg->motor_ki               = MOTOR_DEFAULT_KI;
    cfg->wheel_diam_cm          = ROBOT_WHEEL_DIAM_CM;
    cfg->track_width_cm         = ROBOT_TRACK_WIDTH_CM;
    cfg->encoder_ppr            = ROBOT_ENCODER_PPR;
    cfg->tof_scan_max_mm        = (uint16_t)TOF_SCAN_MAX_MM;
    cfg->yaw_invert             = (uint8_t)IMU_DEFAULT_YAW_INVERT;
    cfg->reserved               = 0U;
    cfg->scan_trigger_x_cm      = FSM_SCAN_TRIGGER_X_CM;
    cfg->pp_lookahead_offset_cm = PP_LOOKAHEAD_OFFSET_CM;
    cfg->crc32                  = compute_cfg_crc(cfg);
}

bool RuntimeConfig_LoadFromFlash(RuntimeConfig_t *cfg)
{
    if (cfg == NULL) return false;
#ifdef STM32F411xE
    RuntimeConfig_t tmp;
    memcpy(&tmp, (const void *)RUNTIME_CFG_FLASH_ADDR, sizeof(RuntimeConfig_t));
    if (tmp.magic != RUNTIME_CFG_MAGIC || tmp.version != RUNTIME_CFG_VERSION) {
        return false;
    }
    if (compute_cfg_crc(&tmp) != tmp.crc32) {
        return false;
    }
    if (!is_valid_range(&tmp)) {
        return false;
    }
    *cfg = tmp;
    return true;
#else
    (void)cfg;
    return false;
#endif
}

void RuntimeConfig_Init(void)
{
    if (!RuntimeConfig_LoadFromFlash(&g_cfg)) {
        RuntimeConfig_LoadDefaults(&g_cfg);
    }
}

bool RuntimeConfig_SaveToFlash(const RuntimeConfig_t *cfg)
{
    if (cfg == NULL || !is_valid_range(cfg)) {
        return false;
    }
#ifdef STM32F411xE
    RuntimeConfig_t staged = *cfg;
    staged.magic    = RUNTIME_CFG_MAGIC;
    staged.version  = RUNTIME_CFG_VERSION;
    staged.reserved = 0U;
    staged.crc32    = compute_cfg_crc(&staged);

    if (HAL_FLASH_Unlock() != HAL_OK) {
        return false;
    }

    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0U;
    memset(&erase_init, 0, sizeof(erase_init));
    erase_init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    erase_init.Sector       = FLASH_SECTOR_7;
    erase_init.NbSectors    = 1U;

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK || sector_error != 0xFFFFFFFFU) {
        (void)HAL_FLASH_Lock();
        return false;
    }

    const uint32_t *words = (const uint32_t *)&staged;
    size_t word_count = sizeof(RuntimeConfig_t) / sizeof(uint32_t);
    for (size_t i = 0; i < word_count; i++) {
        uint32_t addr = RUNTIME_CFG_FLASH_ADDR + (uint32_t)(i * sizeof(uint32_t));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[i]) != HAL_OK) {
            (void)HAL_FLASH_Lock();
            return false;
        }
    }

    (void)HAL_FLASH_Lock();
    return (memcmp((const void *)RUNTIME_CFG_FLASH_ADDR, &staged, sizeof(RuntimeConfig_t)) == 0);
#else
    return false;
#endif
}

bool RuntimeConfig_SetParam(const char *key, float value)
{
    if (key == NULL || !isfinite(value)) {
        return false;
    }

    if (key_equals_nocase(key, "VALL")) {
        if (value < 10.0f || value > 30.0f) return false;
        for (int i = 0; i < 5; i++) {
            g_cfg.v_target[i] = value;
        }
        return true;
    }
    if (key_equals_nocase(key, "V1")) {
        if (value < 10.0f || value > 30.0f) return false;
        g_cfg.v_target[0] = value;
        return true;
    }
    if (key_equals_nocase(key, "V2")) {
        if (value < 10.0f || value > 30.0f) return false;
        g_cfg.v_target[1] = value;
        return true;
    }
    if (key_equals_nocase(key, "V3")) {
        if (value < 10.0f || value > 30.0f) return false;
        g_cfg.v_target[2] = value;
        return true;
    }
    if (key_equals_nocase(key, "V4")) {
        if (value < 10.0f || value > 30.0f) return false;
        g_cfg.v_target[3] = value;
        return true;
    }
    if (key_equals_nocase(key, "V5")) {
        if (value < 10.0f || value > 30.0f) return false;
        g_cfg.v_target[4] = value;
        return true;
    }
    if (key_equals_nocase(key, "VDET")) {
        if (value < 10.0f || value > 30.0f) return false;
        g_cfg.v_detour_max = value;
        return true;
    }
    if (key_equals_nocase(key, "VMAX")) {
        if (value < 15.0f || value > 35.0f) return false;
        g_cfg.v_max = value;
        return true;
    }
    if (key_equals_nocase(key, "ACC")) {
        if (value < 20.0f || value > 150.0f) return false;
        g_cfg.accel_max_cms2 = value;
        return true;
    }
    if (key_equals_nocase(key, "KP")) {
        if (value < 10.0f || value > 600.0f) return false;
        g_cfg.motor_kp = value;
        return true;
    }
    if (key_equals_nocase(key, "KI")) {
        if (value < 0.0f || value > 50.0f) return false;
        g_cfg.motor_ki = value;
        return true;
    }
    if (key_equals_nocase(key, "WD")) {
        if (value < 2.5f || value > 8.0f) return false;
        g_cfg.wheel_diam_cm = value;
        return true;
    }
    if (key_equals_nocase(key, "WB")) {
        if (value < 6.0f || value > 16.0f) return false;
        g_cfg.track_width_cm = value;
        return true;
    }
    if (key_equals_nocase(key, "PPR")) {
        if (value < 100.0f || value > 10000.0f) return false;
        g_cfg.encoder_ppr = value;
        return true;
    }
    if (key_equals_nocase(key, "TOF")) {
        if (value < 150.0f || value > 800.0f) return false;
        g_cfg.tof_scan_max_mm = (uint16_t)(value + 0.5f);
        return true;
    }
    if (key_equals_nocase(key, "SCANX")) {
        if (value < 0.0f || value > 120.0f) return false;
        g_cfg.scan_trigger_x_cm = value;
        return true;
    }
    if (key_equals_nocase(key, "LD")) {
        if (value < 4.0f || value > 12.0f) return false;
        g_cfg.pp_lookahead_offset_cm = value;
        return true;
    }
    if (key_equals_nocase(key, "YAWINV")) {
        int iv = (int)(value + (value >= 0.0f ? 0.5f : -0.5f));
        if (iv != 0 && iv != 1) return false;
        g_cfg.yaw_invert = (uint8_t)iv;
        return true;
    }

    return false;
}

float RuntimeConfig_CmPerTick(void)
{
    if (g_cfg.encoder_ppr <= 1.0f || g_cfg.wheel_diam_cm <= 0.1f) {
        return ROBOT_CM_PER_TICK;
    }
    return (ROBOT_PI_F * g_cfg.wheel_diam_cm) / g_cfg.encoder_ppr;
}
