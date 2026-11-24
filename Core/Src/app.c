/**
 * ============================================================================
 * @file    app.c
 * @brief   Bo dieu phoi tich hop Tang Thap (test_cheesy) + Tang Cao (Tu hanh)
 * ============================================================================
 * - Mac dinh khi cap nguon: active_run = 2 (Che do TU HANH hop nhat: bam nut
 *   PA11 la chay tu O 1 -> O 5, bat 3x VL53L0X quet map tu O 3, van toc
 *   v_target giua cac lan chay giong nhau & hieu chinh duoc qua BLE / CubeMonitor).
 * - Van giu nguyen Telemetry BLE ($TEL, $EVT) de giam sat toa do EKF & ToF
 *   tren dien thoai/PC, va cho phep chuyen RUN:1 lai tay khi can test phan cung.
 * ============================================================================
 */

#include "app.h"
#include "ram_monitor.h"
#include "robot_config.h"
#include "robot_interface.h"
#include "robot_hw_bridge.h"
#include "mission_fsm.h"
#include <math.h>
#include <stdlib.h>

/* ================= External HAL handles (from CubeMX main.c) ================= */
extern ADC_HandleTypeDef  hadc1;
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;
extern TIM_HandleTypeDef  htim2;
extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim10;
extern UART_HandleTypeDef huart2;

/* ================= Timing & IRQ priorities ================= */
#define CTRL_TICK_MS        ROBOT_CTRL_PERIOD_MS   /* 20ms (50Hz) Tang Cao + EKF */
#define BAT_READ_PERIOD_MS  200U
#define IMU_TICK_MS         5U                     /* 200Hz MPU6050              */
#define IRQ_PRIO_MOTOR      0U                     /* TIM10 100Hz PID: CAO NHAT  */
#define IRQ_PRIO_COMM       5U                     /* UART2 / DMA BLE            */

/* ================= Low-level drivers (Friend's code) ================= */
static uint32_t last_ctrl_tick = 0U;
static uint32_t last_bat_tick  = 0U;
static uint32_t last_imu_tick  = 0U;
static uint32_t last_tof_tick  = 0U;

VL53L0X_Device sensors[3];
uint16_t d_left = VL53L0X_OUT_OF_RANGE;
uint16_t d_front = VL53L0X_OUT_OF_RANGE;
uint16_t d_right = VL53L0X_OUT_OF_RANGE;

Motor_Handle_t g_m0; /* Left  (TIM2, TIM4_CH3, PB12/PB13) */
Motor_Handle_t g_m1; /* Right (TIM3, TIM4_CH4, PB14/PB15) */

/* Tham so PI dong co (giu nguyen mac dinh cua ban ban, co the tinh chinh qua CubeMonitor) */
volatile float g_m0_kp = 200.0f;
volatile float g_m0_ki = 0.0f;
volatile float g_m1_kp = 200.0f;
volatile float g_m1_ki = 0.0f;

/* Van toc mong muon v_target cho O 1..O 5 (chinh live qua CubeMonitor hoac BLE VSEG) */
volatile float g_v_target_cms[TRACK_SEG_COUNT] = {
    V_TARGET_O1_CMS,
    V_TARGET_O2_CMS,
    V_TARGET_O3_CMS,
    V_TARGET_O4_CMS,
    V_TARGET_O5_CMS
};
static float s_prev_v_target_cms[TRACK_SEG_COUNT] = {
    V_TARGET_O1_CMS,
    V_TARGET_O2_CMS,
    V_TARGET_O3_CMS,
    V_TARGET_O4_CMS,
    V_TARGET_O5_CMS
};

IMU_t g_imu;

/* ================= Mode & BLE state ================= */
/* Mac dinh active_run = 2 (Tu hanh), khong can BLE de chay thi */
uint8_t active_run = 2U;
RobotTelemetry_t telem_packet;

/* Lenh lai tay (chi dung khi gui BLE RUN:1 de test dong co) */
static volatile int16_t s_manual_v_mmps   = 0;
static volatile int16_t s_manual_w_mradps = 0;
static bool             s_drive_timed     = false;
static uint32_t         s_drive_end_tick  = 0U;

/* Co lenh BLE gui sang Tang Cao */
static bool s_ble_req_start      = false;
static bool s_ble_req_estop      = false;
static bool s_ble_req_reset      = false;
static bool s_btn_short_pending  = false;
static bool s_btn_long_pending   = false;
static uint8_t s_last_reported_fsm = 0xFFU;
static uint8_t s_last_reported_obs = 0U;
static bool    s_imu_calib_ok      = false;

/* ================= Private prototypes ================= */
static void irq_priority_config(void);
static void motor_config(void);
static void battery_config(void);
static void imu_config(void);
static void tof_config(void);
static void telemetry_update(void);
static void emit_fsm_events(void);

static bool on_ble_drive(int16_t v_mmps, int16_t w_mradps, uint16_t duration_ms);
static bool on_ble_obstacle(uint8_t zone, uint8_t pos);
static bool on_ble_mode(uint8_t run_id);
static void on_ble_zero(void);
static void on_ble_stop(void);
static bool on_ble_vtarget(uint8_t seg_1based, int16_t v_cms);

static const BLE_Callbacks_t ble_cbs = {
    .on_drive        = on_ble_drive,
    .on_set_obstacle = on_ble_obstacle,
    .on_set_mode     = on_ble_mode,
    .on_zero         = on_ble_zero,
    .on_stop         = on_ble_stop,
    .on_set_vtarget  = on_ble_vtarget
};

/* ================= Init & Main Loop ================= */

void app_init(void)
{
    irq_priority_config();
    motor_config();
    battery_config();
    imu_config();
    tof_config();

    /* Ket noi cau noi phan cung Tang Thap <-> Tang Cao */
    RobotHwHandles_t hw = {
        .motor_left  = &g_m0,
        .motor_right = &g_m1,
        .imu         = &g_imu,
        .tof_sensors = sensors,
        .btn_port    = BUTT_GPIO_Port,
        .btn_pin     = BUTT_Pin
    };
    RobotBridge_Init(&hw);

    /* Khoi tao Tang Cao: EKF + C-Space 50x74 + Obstacle Mapper + A* + Pure Pursuit + FSM */
    Mission_Init();
    Mission_ResetAll(g_imu.initialized != 0U, g_imu.yaw);

    /* Doc dien ap pin lan dau de goi $TEL dau tien khong bao 0.00V */
    input_voltage = Battery_ReadVoltage(8);

    BLE_Telemetry_Init(&huart2, &ble_cbs);
    RAM_Monitor_Init(BLE_Telemetry_SendBytes);
    (void)BLE_Telemetry_SendLine("$EVT,IMU,%s", s_imu_calib_ok ? "OK" : "UNCAL");

    uint32_t now = HAL_GetTick();
    last_ctrl_tick = now;
    last_bat_tick  = now;
    last_imu_tick  = now;
    last_tof_tick  = now;
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();

    /* 1. Xu ly truyen thong BLE & RAM Monitor (non-blocking) */
    BLE_Telemetry_Process();
    RAM_Monitor_StreamDue(now);

    /* 2. Quet nut bam PA11 lien tuc (bat su kien nhan nha & nhan giu >= 1.2s) */
    bool short_evt = false, long_evt = false;
    RobotBridge_PollButton(now, &short_evt, &long_evt);
    if (short_evt) s_btn_short_pending = true;
    if (long_evt)  s_btn_long_pending  = true;

    /* 3. Doc dien ap pin & den bao PC13 */
    if ((now - last_bat_tick) >= BAT_READ_PERIOD_MS) {
        last_bat_tick = now;
        input_voltage = Battery_ReadVoltage(8);
    }
    Battery_Update_Indicator();

    /* 4. Cap nhat IMU MPU6050 tan so cao 200Hz (moi 5ms) */
    if ((now - last_imu_tick) >= IMU_TICK_MS) {
        last_imu_tick = now;
        (void)IMU_Update(&g_imu);
    }

    /* 5. Cap nhat 3 cam bien Laser ToF VL53L0X non-blocking (moi 20ms)
     *    Chi bao tuoi (fresh) khi VL53L0X_Update() thuc su tra ve mau do moi. */
    if ((now - last_tof_tick) >= 20U) {
        last_tof_tick = now;
        for (uint8_t i = 0U; i < 3U; i++) {
            if (VL53L0X_Update(&sensors[i]) != 0U) {
                RobotBridge_NotifyToFFresh(i, now);
            }
        }
        d_left  = sensors[0].valid ? sensors[0].distance_mm : VL53L0X_OUT_OF_RANGE;
        d_front = sensors[1].valid ? sensors[1].distance_mm : VL53L0X_OUT_OF_RANGE;
        d_right = sensors[2].valid ? sensors[2].distance_mm : VL53L0X_OUT_OF_RANGE;
    }

    /* 6. Chu ky dieu khien chinh 50Hz (20ms): EKF + FSM + PurePursuit + Motor */
    if ((now - last_ctrl_tick) >= CTRL_TICK_MS) {
        uint32_t dt_ms = now - last_ctrl_tick;
        last_ctrl_tick = now;
        float dt_s = (float)dt_ms * 0.001f;

        /* Cho phep CubeMonitor chinh nhanh Kp/Ki va v_target an toan ngat khi test */
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        g_m0.pi.Kp = g_m0_kp;
        g_m0.pi.Ki = g_m0_ki;
        g_m1.pi.Kp = g_m1_kp;
        g_m1.pi.Ki = g_m1_ki;
        __set_PRIMASK(primask);

        for (uint8_t i = 0U; i < TRACK_SEG_COUNT; i++) {
            float cur_v = g_v_target_cms[i];
            if (cur_v != s_prev_v_target_cms[i]) {
                if (Mission_SetSegmentTargetSpeed((TrackSegment_t)i, cur_v)) {
                    float clamped = Mission_GetSegmentTargetSpeed((TrackSegment_t)i);
                    g_v_target_cms[i]      = clamped;
                    s_prev_v_target_cms[i] = clamped;
                }
            }
        }

        /* Doc dong bo Encoder + IMU + ToF + Pin + Nut bam */
        RobotSensorFrame_t frame;
        RobotBridge_CaptureSensors(now, dt_s, input_voltage,
                                   s_btn_short_pending, s_btn_long_pending,
                                   &frame);
        s_btn_short_pending = false;
        s_btn_long_pending  = false;

        frame.ble_cmd_start = s_ble_req_start;
        frame.ble_cmd_estop = s_ble_req_estop;
        frame.ble_cmd_reset = s_ble_req_reset;
        s_ble_req_start     = false;
        s_ble_req_estop     = false;
        s_ble_req_reset     = false;

        RobotDriveCmd_t drive_cmd;
        if (active_run == 2U) {
            /* CHE DO TU HANH: EKF + Quet 3x ToF + A* + Pure Pursuit + FSM */
            Mission_Step(&frame, &drive_cmd);
            RobotBridge_ApplyDriveCmd(&drive_cmd, dt_s);
            emit_fsm_events();
        } else {
            /* CHE DO LAI TAY (RUN:1): Giu FSM o IDLE nhung van cap nhat EKF lien tuc */
            frame.btn_short_press = false;
            frame.btn_long_press  = false;
            frame.ble_cmd_start   = false;
            Mission_Step(&frame, &drive_cmd);

            if (s_drive_timed && (int32_t)(now - s_drive_end_tick) >= 0) {
                s_drive_timed     = false;
                s_manual_v_mmps   = 0;
                s_manual_w_mradps = 0;
                BLE_Telemetry_SendLine("$EVT,DRV_DONE");
            }
            drive_cmd.v_linear_cms   = (float)s_manual_v_mmps * 0.1f;
            drive_cmd.omega_rads     = (float)s_manual_w_mradps * 0.001f;
            drive_cmd.brake_stop     = (s_manual_v_mmps == 0 && s_manual_w_mradps == 0);
            drive_cmd.robot_is_still = drive_cmd.brake_stop;
            RobotBridge_ApplyDriveCmd(&drive_cmd, dt_s);
        }
    }

    /* 7. Gui goi Telemetry BLE ($TEL) chua toa do EKF that + ToF + van toc banh */
    telemetry_update();
}

/* ================= HAL Interrupt Callbacks ================= */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM10) {
        (void)motor_update_loop(&g_m0);
        (void)motor_update_loop(&g_m1);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        BLE_Telemetry_RxEventCallback(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        BLE_Telemetry_ErrorCallback();
    }
}

/* ================= BLE Handlers ================= */

static bool on_ble_drive(int16_t v_mmps, int16_t w_mradps, uint16_t duration_ms)
{
    if (active_run != 1U) {
        return false;
    }
    /* Gioi han toc do lai tay cung <= 300 mm/s (30 cm/s) */
    if (v_mmps > 300)  v_mmps = 300;
    if (v_mmps < -300) v_mmps = -300;

    s_manual_v_mmps   = v_mmps;
    s_manual_w_mradps = w_mradps;
    if (duration_ms > 0U) {
        s_drive_timed    = true;
        s_drive_end_tick = HAL_GetTick() + duration_ms;
    } else {
        s_drive_timed = false;
    }
    return true;
}

static void on_ble_stop(void)
{
    s_manual_v_mmps   = 0;
    s_manual_w_mradps = 0;
    s_drive_timed     = false;
    s_ble_req_estop   = true;

    RobotDriveCmd_t stop_cmd = { 0.0f, 0.0f, true, true };
    RobotBridge_ApplyDriveCmd(&stop_cmd, ROBOT_CTRL_DT_S);
}

static bool on_ble_obstacle(uint8_t zone, uint8_t pos)
{
    /* Tuong thich 100% preset MODE_A..MODE_E trong map_config.py cua ban ban */
    static const float k_preset_xy[5][2] = {
        { 115.0f, 100.0f }, /* pos 1: MODE_A - Giua T13 - T14 */
        { 125.0f,  65.0f }, /* pos 2: MODE_B - Ngay duoi T14  */
        { 125.0f,  30.0f }, /* pos 3: MODE_C - Hanh lang trai */
        { 155.0f,  25.0f }, /* pos 4: MODE_D - Khe T15 - T17  */
        { 145.0f,   5.0f }  /* pos 5: MODE_E - Sat cua O 5    */
    };
    if (zone == 3U && pos == 1U) {
        return Mission_InjectObstacle(102.0f, 95.0f);
    }
    if ((zone == 4U || zone == 5U) && pos >= 1U && pos <= 5U) {
        return Mission_InjectObstacle(k_preset_xy[pos - 1U][0], k_preset_xy[pos - 1U][1]);
    }
    return false;
}

static bool on_ble_mode(uint8_t run_id)
{
    if (run_id != 1U && run_id != 2U && run_id != 3U) {
        return false;
    }
    RobotDriveCmd_t stop_cmd = { 0.0f, 0.0f, true, true };
    RobotBridge_ApplyDriveCmd(&stop_cmd, ROBOT_CTRL_DT_S);
    s_manual_v_mmps   = 0;
    s_manual_w_mradps = 0;
    s_drive_timed     = false;

    if (run_id == 1U) {
        active_run = 1U;
        s_ble_req_reset = true;
    } else {
        /* RUN:2 (va alias RUN:3) = Chay tu hanh hop nhat */
        active_run = 2U;
        s_ble_req_start = true;
    }
    return true;
}

static bool on_ble_vtarget(uint8_t seg_1based, int16_t v_cms)
{
    if (seg_1based == 0U) {
        if (!Mission_SetUniformTargetSpeed((float)v_cms)) {
            return false;
        }
        float clamped = Mission_GetSegmentTargetSpeed(TRACK_SEG_O1_START_NARROW);
        for (uint8_t i = 0U; i < TRACK_SEG_COUNT; i++) {
            g_v_target_cms[i]      = clamped;
            s_prev_v_target_cms[i] = clamped;
        }
        return true;
    }
    if (seg_1based <= (uint8_t)TRACK_SEG_COUNT) {
        TrackSegment_t seg = (TrackSegment_t)(seg_1based - 1U);
        if (!Mission_SetSegmentTargetSpeed(seg, (float)v_cms)) {
            return false;
        }
        float clamped = Mission_GetSegmentTargetSpeed(seg);
        g_v_target_cms[seg]      = clamped;
        s_prev_v_target_cms[seg] = clamped;
        return true;
    }
    return false;
}

static void on_ble_zero(void)
{
    IMU_ResetYaw(&g_imu);
    RobotBridge_ResetDriveRamp();
    RobotBridge_ResetEncoderReference();
    Mission_ResetAll(g_imu.initialized != 0U, g_imu.yaw);
    s_ble_req_reset = false;
}

/* ================= Telemetry & Events ================= */

static void emit_fsm_events(void)
{
    RobotStatus_t st = Mission_GetStatus();
    if (st.fsm_state != s_last_reported_fsm) {
        s_last_reported_fsm = st.fsm_state;
        BLE_Telemetry_SendLine("$EVT,FSM,%u,%ld", (unsigned)st.fsm_state, (long)st.estimated_score);
    }
    /* Dong bo lai bo dem khi mapper bi xoa (ZERO / chuyen Luot 2 / Reset) */
    if (st.dynamic_obs_count < s_last_reported_obs) {
        s_last_reported_obs = st.dynamic_obs_count;
    }
    while (s_last_reported_obs < st.dynamic_obs_count) {
        const DynamicObstacle_t *obs =
            ObstacleMapper_GetObstacle(Mission_GetMapper(), s_last_reported_obs);
        s_last_reported_obs++;
        if (obs != NULL) {
            BLE_Telemetry_SendLine("$EVT,OBS,%d,%d",
                                   (int)(obs->center_x_cm + 0.5f),
                                   (int)(obs->center_y_cm + 0.5f));
        }
    }
}

static void telemetry_update(void)
{
    if (!BLE_Telemetry_ReportDue()) {
        return;
    }

    RobotPose_t pose = Mission_GetPose();

    telem_packet.current_mode    = active_run;
    telem_packet.v_battery       = input_voltage;
    telem_packet.x_pos           = pose.x_cm;
    telem_packet.y_pos           = pose.y_cm;
    telem_packet.heading         = pose.theta_rad * ROBOT_RAD2DEG_F;
    telem_packet.dist_left       = d_left;
    telem_packet.dist_front      = d_front;
    telem_packet.dist_right      = d_right;
    telem_packet.vright_expected = g_m1.state.dq_cmd_rads;
    telem_packet.vleft_expected  = g_m0.state.dq_cmd_rads;
    telem_packet.vright_reality  = g_m1.state.dq_state_rads;
    telem_packet.vleft_reality   = g_m0.state.dq_state_rads;

    (void)BLE_Telemetry_SendReport(&telem_packet);
}

/* ================= Peripheral Configuration (Preserved from friend's code) ================= */

static void irq_priority_config(void)
{
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, IRQ_PRIO_MOTOR, 0);
    HAL_NVIC_SetPriority(USART2_IRQn,        IRQ_PRIO_COMM,  0);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn,  IRQ_PRIO_COMM,  0);
}

static void motor_config(void)
{
    Motor_Config_t m0_cfg = {
        .htim_enc            = &htim2,
        .is_reversed_encoder = false,
        .htim_pwm            = &htim4,
        .pwm_channel         = TIM_CHANNEL_3,
        .dir_port_a          = GPIOB,
        .dir_pin_a           = GPIO_PIN_12,
        .dir_port_b          = GPIOB,
        .dir_pin_b           = GPIO_PIN_13,
        .is_reversed_dir     = false,
        .pulses_per_rev      = ROBOT_ENCODER_PPR,
        .wheel_diameter_m    = ROBOT_WHEEL_DIAM_M,
        .control_period_s    = 0.01f
    };
    Motor_PID_Params_t m0_pid_cfg = {
        .Kp             = g_m0_kp,
        .Ki             = g_m0_ki,
        .max_integral   = 200.0f,
        .max_output     = 999.0f,
        .max_speed_rads = ROBOT_MAX_WHEEL_RADS
    };

    Motor_Config_t m1_cfg = {
        .htim_enc            = &htim3,
        .is_reversed_encoder = true,
        .htim_pwm            = &htim4,
        .pwm_channel         = TIM_CHANNEL_4,
        .dir_port_a          = GPIOB,
        .dir_pin_a           = GPIO_PIN_14,
        .dir_port_b          = GPIOB,
        .dir_pin_b           = GPIO_PIN_15,
        .is_reversed_dir     = false,
        .pulses_per_rev      = ROBOT_ENCODER_PPR,
        .wheel_diameter_m    = ROBOT_WHEEL_DIAM_M,
        .control_period_s    = 0.01f
    };
    Motor_PID_Params_t m1_pid_cfg = {
        .Kp             = g_m1_kp,
        .Ki             = g_m1_ki,
        .max_integral   = 520.0f,
        .max_output     = 999.0f,
        .max_speed_rads = ROBOT_MAX_WHEEL_RADS
    };

    (void)motor_init(&g_m0, &m0_cfg, &m0_pid_cfg);
    (void)motor_enable(&g_m0);

    (void)motor_init(&g_m1, &m1_cfg, &m1_pid_cfg);
    (void)motor_enable(&g_m1);

    /* Enable STBY - TB6612 */
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

    /* Enable TIM10 - 100Hz Motor PI Loop */
    HAL_TIM_Base_Start_IT(&htim10);
}

static void imu_config(void)
{
    s_imu_calib_ok = false;
    IMU_Status imu_st = IMU_Init(&g_imu, &hi2c2, IMU_ADDR_AD0_LOW,
                                 GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_3);
    if (imu_st == IMU_OK) {
        for (uint8_t attempt = 0U; attempt < 2U; attempt++) {
            HAL_Delay(300);
            if (IMU_CalibrateGyro(&g_imu, IMU_CAL_SAMPLES) == IMU_OK) {
                s_imu_calib_ok = true;
                break;
            }
        }
        IMU_ResetYaw(&g_imu);
    }
}

static void tof_config(void)
{
    VL53L0X_Create(&sensors[0], &hi2c1, XSHUT01_GPIO_Port, XSHUT01_Pin, 0x54);
    VL53L0X_Create(&sensors[1], &hi2c1, XSHUT02_GPIO_Port, XSHUT02_Pin, 0x56);
    VL53L0X_Create(&sensors[2], &hi2c1, XSHUT03_GPIO_Port, XSHUT03_Pin, 0x58);

    if (VL53L0X_InitAll(sensors, 3U) != VL53L0X_OK) {
        d_left  = VL53L0X_OUT_OF_RANGE;
        d_front = VL53L0X_OUT_OF_RANGE;
        d_right = VL53L0X_OUT_OF_RANGE;
    }

    for (uint8_t i = 0U; i < 3U; i++) {
        if (sensors[i].initialized) {
            /* Dat timing budget 20ms (fast mode) truoc khi chay continuous back-to-back
             * de dat dung tan so lay mau ~50Hz khop chu ky Tang Cao 20ms */
            (void)VL53L0X_SetTimingBudget(&sensors[i], 20000U);
            (void)VL53L0X_StartContinuous(&sensors[i], 0U);
        }
    }
}

static void battery_config(void)
{
    Battery_Config_t bat_cfg = {
        .hadc       = &hadc1,
        .led_port   = GPIOC,
        .led_pin    = GPIO_PIN_13,
        .vref       = 3.3f,
        .r_top_k    = 10.0f,
        .r_bottom_k = 5.1f,
        .v_min      = 7.4f,
        .v_max      = 8.4f
    };
    Battery_Init(&bat_cfg);
}
