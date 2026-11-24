/**
 * ============================================================================
 * @file    robot_hw_bridge.h
 * @brief   Cau noi phan cung Tang Thap (STM32 HAL / friend's drivers)
 *          <-> Tang Cao (HighLevel/ EKF + A* + PurePursuit + FSM)
 * ============================================================================
 */

#ifndef ROBOT_HW_BRIDGE_H
#define ROBOT_HW_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include "robot_interface.h"
#include "motor_api.h"
#include "imu.h"
#include "vl53l0x.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Motor_Handle_t *motor_left;
    Motor_Handle_t *motor_right;
    IMU_t          *imu;
    VL53L0X_Device *tof_sensors; /* Mang 3 cam bien: [0]=Left, [1]=Front, [2]=Right */
    GPIO_TypeDef   *btn_port;
    uint16_t        btn_pin;
} RobotHwHandles_t;

/* Khoi tao cau noi phan cung */
void RobotBridge_Init(const RobotHwHandles_t *handles);

/* Dong bo lai bo dem xung encoder (goi khi reset toa do / bat dau luot moi) */
void RobotBridge_ResetEncoderReference(void);

/* Reset trang thai ramp van toc ve 0 (khi dung khan cap hoac doi che do) */
void RobotBridge_ResetDriveRamp(void);

/* Quet nut bam PA11 (chong rung 40ms, phan biet nhan nha < 1.2s va nhan giu >= 1.2s) */
void RobotBridge_PollButton(uint32_t now_ms, bool *out_short_press, bool *out_long_press);

/* Bao co mau do ToF moi tu VL53L0X_Update() cho kenh 0=Left, 1=Front, 2=Right */
void RobotBridge_NotifyToFFresh(uint8_t sensor_idx, uint32_t now_ms);

/* Dong goi toan bo cam bien (Encoder atomic + IMU + 3x ToF + Pin) vao SensorFrame */
void RobotBridge_CaptureSensors(uint32_t now_ms,
                                float dt_s,
                                float v_battery,
                                bool btn_short,
                                bool btn_long,
                                RobotSensorFrame_t *out_frame);

/* Ap dung lenh lai (v_cms, omega_rads) voi gioi han gia toc (slew) va bao hoa banh 30cm/s */
void RobotBridge_ApplyDriveCmd(const RobotDriveCmd_t *cmd, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_HW_BRIDGE_H */
