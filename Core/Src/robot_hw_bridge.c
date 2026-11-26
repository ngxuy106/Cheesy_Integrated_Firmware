/**
 * ============================================================================
 * @file    robot_hw_bridge.c
 * @brief   Hien thuc cau noi an toan ngat (ISR-safe) giua driver Tang Thap
 *          cua ban ban (motor_api, imu, vl53l0x) va Tang Cao (Mission FSM)
 * ============================================================================
 */

#include "robot_hw_bridge.h"
#include "robot_config.h"
#include <math.h>
#include <string.h>

#define BTN_DEBOUNCE_MS     40U
#define BTN_LONG_PRESS_MS   1200U

static RobotHwHandles_t s_hw;
static bool     s_inited          = false;
static int32_t  s_prev_ticks_left = 0;
static int32_t  s_prev_ticks_right = 0;

/* Ramp van toc chong giat / truot banh */
static float    s_cur_v_cms       = 0.0f;
static float    s_cur_w_rads      = 0.0f;

/* Trang thai chong rung nut bam PA11 (Active LOW, Pull-up) */
static bool     s_btn_raw_prev    = false;
static bool     s_btn_stable      = false;
static bool     s_btn_long_fired  = false;
static uint32_t s_btn_debounce_ms = 0U;
static uint32_t s_btn_press_start = 0U;

/* Theo doi do tuoi mau do 3x VL53L0X (tranh dem lap mau cu va phat hien treo I2C) */
static bool     s_tof_fresh_pending[3] = { false, false, false };
static bool     s_tof_ever_fresh[3]    = { false, false, false };
static uint32_t s_tof_last_fresh_ms[3] = { 0U, 0U, 0U };

static float slew_limit(float cur, float target, float max_step)
{
    float diff = target - cur;
    if (diff >  max_step) return cur + max_step;
    if (diff < -max_step) return cur - max_step;
    return target;
}

void RobotBridge_Init(const RobotHwHandles_t *handles)
{
    if (handles == NULL) {
        return;
    }
    s_hw             = *handles;
    s_cur_v_cms      = 0.0f;
    s_cur_w_rads     = 0.0f;
    s_btn_raw_prev   = false;
    s_btn_stable     = false;
    s_btn_long_fired = false;
    for (uint8_t i = 0U; i < 3U; i++) {
        s_tof_fresh_pending[i] = false;
        s_tof_ever_fresh[i]    = false;
        s_tof_last_fresh_ms[i] = 0U;
    }
    s_inited         = true;

    RobotBridge_ResetEncoderReference();
}

void RobotBridge_NotifyToFFresh(uint8_t sensor_idx, uint32_t now_ms)
{
    if (sensor_idx < 3U) {
        s_tof_fresh_pending[sensor_idx] = true;
        s_tof_ever_fresh[sensor_idx]    = true;
        s_tof_last_fresh_ms[sensor_idx] = now_ms;
    }
}

void RobotBridge_ResetEncoderReference(void)
{
    if (!s_inited || s_hw.motor_left == NULL || s_hw.motor_right == NULL) {
        return;
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_prev_ticks_left  = s_hw.motor_left->state.total_ticks;
    s_prev_ticks_right = s_hw.motor_right->state.total_ticks;
    __set_PRIMASK(primask);
}

void RobotBridge_ResetDriveRamp(void)
{
    s_cur_v_cms  = 0.0f;
    s_cur_w_rads = 0.0f;
}

void RobotBridge_PollButton(uint32_t now_ms, bool *out_short_press, bool *out_long_press)
{
    if (out_short_press != NULL) *out_short_press = false;
    if (out_long_press  != NULL) *out_long_press  = false;
    if (!s_inited || s_hw.btn_port == NULL) {
        return;
    }

    /* Nut PA11 noi GND voi Pull-up -> RESET la dang nhan */
    bool raw_pressed = (HAL_GPIO_ReadPin(s_hw.btn_port, s_hw.btn_pin) == GPIO_PIN_RESET);

    if (raw_pressed != s_btn_raw_prev) {
        s_btn_raw_prev    = raw_pressed;
        s_btn_debounce_ms = now_ms;
    }

    if ((now_ms - s_btn_debounce_ms) >= BTN_DEBOUNCE_MS) {
        if (raw_pressed && !s_btn_stable) {
            /* Canh nhan xuong on dinh */
            s_btn_stable      = true;
            s_btn_long_fired  = false;
            s_btn_press_start = now_ms;
        } else if (!raw_pressed && s_btn_stable) {
            /* Canh nha nut: chi bao short press neu chua kich hoat long press */
            s_btn_stable = false;
            if (!s_btn_long_fired && out_short_press != NULL) {
                *out_short_press = true;
            }
        }
    }

    if (s_btn_stable && !s_btn_long_fired &&
        (now_ms - s_btn_press_start) >= BTN_LONG_PRESS_MS) {
        s_btn_long_fired = true;
        if (out_long_press != NULL) {
            *out_long_press = true;
        }
    }
}

void RobotBridge_CaptureSensors(uint32_t now_ms,
                                float dt_s,
                                float v_battery,
                                bool btn_short,
                                bool btn_long,
                                RobotSensorFrame_t *out_frame)
{
    if (out_frame == NULL) {
        return;
    }
    memset(out_frame, 0, sizeof(*out_frame));

    out_frame->timestamp_ms    = now_ms;
    out_frame->dt_s            = dt_s;
    out_frame->v_battery       = v_battery;
    out_frame->btn_short_press = btn_short;
    out_frame->btn_long_press  = btn_long;

    if (!s_inited) {
        return;
    }

    /* 1. Doc dong thoi cap xung Encoder Left + Right (tranh lech pha 10ms do TIM10 ISR) */
    if (s_hw.motor_left != NULL && s_hw.motor_right != NULL) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        int32_t cur_l = s_hw.motor_left->state.total_ticks;
        int32_t cur_r = s_hw.motor_right->state.total_ticks;
        float   wl    = s_hw.motor_left->state.dq_state_rads;
        float   wr    = s_hw.motor_right->state.dq_state_rads;
        __set_PRIMASK(primask);

        /* Tru khong dau -> ep signed 32-bit de tranh UB khi tran bo dem */
        out_frame->delta_ticks_left  = (int32_t)((uint32_t)cur_l - (uint32_t)s_prev_ticks_left);
        out_frame->delta_ticks_right = (int32_t)((uint32_t)cur_r - (uint32_t)s_prev_ticks_right);
        out_frame->total_ticks_left  = cur_l;
        out_frame->total_ticks_right = cur_r;
        out_frame->wheel_left_rads   = wl;
        out_frame->wheel_right_rads  = wr;

        s_prev_ticks_left  = cur_l;
        s_prev_ticks_right = cur_r;
    }

    /* 2. Doc du lieu MPU6050 (IMU da cap nhat 200Hz trong app_loop) */
    if (s_hw.imu != NULL && s_hw.imu->initialized && s_hw.imu->last_error == IMU_OK) {
        out_frame->imu_valid   = true;
        out_frame->imu_yaw_deg = s_hw.imu->yaw;
        out_frame->imu_gz_rads = s_hw.imu->gz * ROBOT_DEG2RAD_F;
    } else {
        out_frame->imu_valid   = false;
        out_frame->imu_yaw_deg = 0.0f;
        out_frame->imu_gz_rads = 0.0f;
    }

    /* 3. Doc 3 cam bien Laser ToF VL53L0X: [0]=Left, [1]=Front, [2]=Right
     *    Chi danh dau tuoi neu co mau moi; vo hieu hoa neu cam bien treo > 120ms */
    if (s_hw.tof_sensors != NULL) {
        bool     valids[3];
        uint16_t dists[3];
        for (uint8_t i = 0U; i < 3U; i++) {
            bool fresh = s_tof_fresh_pending[i];
            s_tof_fresh_pending[i] = false;
            out_frame->tof_is_stale[i] = !fresh;

            bool timed_out = (!s_tof_ever_fresh[i]) ||
                             ((now_ms - s_tof_last_fresh_ms[i]) > TOF_STALE_TIMEOUT_MS);
            bool ok = (s_hw.tof_sensors[i].initialized != 0U) &&
                      (s_hw.tof_sensors[i].valid != 0U) &&
                      !timed_out;
            valids[i] = ok;
            dists[i]  = ok ? s_hw.tof_sensors[i].distance_mm : (uint16_t)TOF_OUT_OF_RANGE_MM;
        }
        out_frame->tof_left_valid  = valids[0];
        out_frame->tof_front_valid = valids[1];
        out_frame->tof_right_valid = valids[2];
        out_frame->tof_left_mm     = dists[0];
        out_frame->tof_front_mm    = dists[1];
        out_frame->tof_right_mm    = dists[2];
    }
}

void RobotBridge_ApplyDriveCmd(const RobotDriveCmd_t *cmd, float dt_s)
{
    if (!s_inited || cmd == NULL || s_hw.motor_left == NULL || s_hw.motor_right == NULL) {
        return;
    }
    if (!isfinite(dt_s) || dt_s <= 1e-4f || dt_s > 0.2f) {
        dt_s = ROBOT_CTRL_DT_S;
    }

    /* Cho phep IMU bam theo troi nhiet khi xe dung yen */
    if (s_hw.imu != NULL) {
        IMU_TrackBias(s_hw.imu, cmd->robot_is_still ? 1U : 0U);
    }

    /* Chong nhiem NaN/Inf: neu lenh dieu khien hong, phanh dung an toan ngay lap tuc */
    bool invalid_cmd = !isfinite(cmd->v_linear_cms) || !isfinite(cmd->omega_rads);
    if (cmd->brake_stop || invalid_cmd) {
        s_cur_v_cms  = 0.0f;
        s_cur_w_rads = 0.0f;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        (void)motor_SetTargetSpeed_Rads(s_hw.motor_left,  0.0f);
        (void)motor_SetTargetSpeed_Rads(s_hw.motor_right, 0.0f);
        __set_PRIMASK(primask);
        return;
    }

    /* 1. Kep van toc muc tieu trong nguong cho phep (max 30 cm/s) */
    float target_v = cmd->v_linear_cms;
    float target_w = cmd->omega_rads;
    if (target_v >  ROBOT_MAX_SPEED_CMS) target_v =  ROBOT_MAX_SPEED_CMS;
    if (target_v < -ROBOT_MAX_SPEED_CMS) target_v = -ROBOT_MAX_SPEED_CMS;
    if (target_w >  ROBOT_MAX_OMEGA_RADS) target_w =  ROBOT_MAX_OMEGA_RADS;
    if (target_w < -ROBOT_MAX_OMEGA_RADS) target_w = -ROBOT_MAX_OMEGA_RADS;

    /* 2. Ramp gia toc chong truot banh khi tang/giam toc */
    s_cur_v_cms  = slew_limit(s_cur_v_cms,  target_v, ROBOT_ACCEL_MAX_CMS2  * dt_s);
    s_cur_w_rads = slew_limit(s_cur_w_rads, target_w, ROBOT_ALPHA_MAX_RADS2 * dt_s);

    /* 3. Dong hoc nguoc xe vi sai: (v, w) -> (v_L, v_R) [cm/s] */
    float half_wb = ROBOT_TRACK_WIDTH_CM * 0.5f;
    float v_left_cms  = s_cur_v_cms - s_cur_w_rads * half_wb;
    float v_right_cms = s_cur_v_cms + s_cur_w_rads * half_wb;

    /* 4. Bao hoa 2 banh theo cung ti le de giu dung ban kinh cua & <= 30 cm/s */
    float peak = fmaxf(fabsf(v_left_cms), fabsf(v_right_cms));
    if (peak > ROBOT_MAX_WHEEL_SPEED_CMS) {
        float scale = ROBOT_MAX_WHEEL_SPEED_CMS / peak;
        v_left_cms  *= scale;
        v_right_cms *= scale;
    }

    float w_left_rads  = v_left_cms  / ROBOT_WHEEL_RADIUS_CM;
    float w_right_rads = v_right_cms / ROBOT_WHEEL_RADIUS_CM;

    /* 5. Nap dong thoi xuong 2 kenh PID motor (TIM10 100Hz) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    (void)motor_SetTargetSpeed_Rads(s_hw.motor_left,  w_left_rads);
    (void)motor_SetTargetSpeed_Rads(s_hw.motor_right, w_right_rads);
    __set_PRIMASK(primask);
}
