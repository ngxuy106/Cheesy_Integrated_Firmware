/**
 * ============================================================================
 * @file    robot_config.h
 * @brief   Bang tham so thong nhat 1 noi cho xe tu hanh Cheesy (STM32F411CEU6)
 * ============================================================================
 * Ghi chu trao doi 2 nguoi (Tang Thap <-> Tang Cao):
 * - Moi thong so co khi duoc dong bo truc tiep voi phan cung thuc te trong
 *   test_cheesy-master (app.c, motor_api.c) va do thuoc kep (kichthuocxe.jpg).
 * - Khi thay doi banh xe, encoder, hoac toc do toi da, CHI CAN SUA TAI DAY.
 * ============================================================================
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 1. THONG SO CO KHI & DONG HOC VI SAI (GROUNDED FROM REAL HARDWARE)
 * ============================================================================ */
#define ROBOT_PI_F                      3.14159265358979323846f
#define ROBOT_DEG2RAD_F                 (ROBOT_PI_F / 180.0f)
#define ROBOT_RAD2DEG_F                 (180.0f / ROBOT_PI_F)

/* Do thuc te tren xe: duong kinh banh 4.5cm, khoang cach tam 2 banh 9.6cm,
 * encoder 4x quadrature = 2800 xung/vong banh (tu app.c & motor_api.c). */
#define ROBOT_WHEEL_DIAM_CM             4.50f
#define ROBOT_WHEEL_RADIUS_CM           (ROBOT_WHEEL_DIAM_CM * 0.5f)
#define ROBOT_WHEEL_DIAM_M              (ROBOT_WHEEL_DIAM_CM * 0.01f)
#define ROBOT_WHEEL_RADIUS_M            (ROBOT_WHEEL_RADIUS_CM * 0.01f)
#define ROBOT_TRACK_WIDTH_CM            9.60f
#define ROBOT_TRACK_WIDTH_M             (ROBOT_TRACK_WIDTH_CM * 0.01f)
#define ROBOT_ENCODER_PPR               2800.0f
#define ROBOT_CM_PER_TICK               ((ROBOT_PI_F * ROBOT_WHEEL_DIAM_CM) / ROBOT_ENCODER_PPR)

/* Kich thuoc phu bi khung xe (thuoc kep do 114.6mm rong x ~100mm dai) */
#define ROBOT_LENGTH_CM                 10.0f
#define ROBOT_WIDTH_CM                  12.0f
#define ROBOT_R_SAFE_CM                 7.5f

/* ============================================================================
 * 2. GIOI HAN VAN TOC & GIA TOC (MAX SPEED ~ 30 CM/S)
 * ============================================================================ */
#define ROBOT_MAX_SPEED_CMS             30.0f   /* Van toc tinh tien toi da (cm/s) */
#define ROBOT_MAX_WHEEL_SPEED_CMS       ROBOT_MAX_SPEED_CMS
#define ROBOT_MAX_WHEEL_RADS            (ROBOT_MAX_WHEEL_SPEED_CMS / ROBOT_WHEEL_RADIUS_CM)
#define ROBOT_MIN_CRUISE_SPEED_CMS      10.0f   /* Van toc toi thieu khi vao cua   */
#define ROBOT_MAX_OMEGA_RADS            2.60f   /* Van toc goc toi da (rad/s)      */
#define ROBOT_PIVOT_OMEGA_RADS          1.60f   /* Toc do xoay tai cho (rad/s)     */

/* Slew-rate limit chong truot banh -> giu EKF Odometry chinh xac */
#define ROBOT_ACCEL_MAX_CMS2            65.0f   /* 0 -> 30 cm/s trong ~0.46s       */
#define ROBOT_ALPHA_MAX_RADS2           6.0f    /* Gia toc goc toi da (rad/s^2)    */

/* Mac dinh PI dong co & chieu Yaw IMU */
#define MOTOR_DEFAULT_KP                200.0f
#define MOTOR_DEFAULT_KI                0.0f
#define IMU_DEFAULT_YAW_INVERT          0U

/* Chu ky vong lap dieu khien Tang Cao (50Hz = 20ms; PID dong co TIM10 = 100Hz) */
#define ROBOT_CTRL_PERIOD_MS            20U
#define ROBOT_CTRL_DT_S                 0.02f

/* ============================================================================
 * 2B. VAN TOC MONG MUON v_target THEO TUNG DOAN O 1..O 5 (HIEU CHINH SAU MOI LAN TEST)
 * - Gop chung 1 che do chay duy nhat (khong chia Luot 1 / Luot 2, toc do giong nhau).
 * - Sua truc tiep 5 hang so duoi day, HOAC doi live qua BLE "CFG:<KEY>,<VAL>"
 *   / "CHINH_THONG_SO.bat" va luu Flash bang "CFG:SAVE".
 * ============================================================================ */
#define V_TARGET_O1_CMS                 20.0f   /* O 1: Khe hep xuat phat          */
#define V_TARGET_O2_CMS                 18.0f   /* O 2: Bo cua trai                */
#define V_TARGET_O3_CMS                 26.0f   /* O 3: Bat 3x ToF quet map        */
#define V_TARGET_O4_CMS                 17.0f   /* O 4: Bo cua phai + quet ToF     */
#define V_TARGET_O5_CMS                 24.0f   /* O 5: Ve dich (<= 30 cm/s)       */
#define V_TARGET_DETOUR_MAX_CMS         17.0f   /* Tran toc do khi ne vat can dong */

/* Diem bat dau bat quet 3x VL53L0X tim vat can dong: ngay khi vao O 3 (x >= 60 cm) */
#define FSM_SCAN_TRIGGER_X_CM           60.0f
#define FSM_SCAN_TRIGGER_Y_CM           55.0f
#define OBS_SCAN_ZONE_MIN_X_CM          FSM_SCAN_TRIGGER_X_CM

/* ============================================================================
 * 3. CAM BIEN 3x VL53L0X LASER ToF (TRAI +45 DEG, TRUOC 0 DEG, PHAI -45 DEG)
 * ============================================================================ */
#define TOF_OUT_OF_RANGE_MM             8190U
#define TOF_MIN_VALID_MM                25U     /* Bo qua nhieu < 2.5cm            */
#define TOF_SCAN_MAX_MM                 420U    /* Tam quet tin cay vat can (42cm) */
#define TOF_EMERGENCY_GUARD_MM          85U     /* Phanh/lui tranh va cham < 8.5cm */
#define TOF_OBS_HALF_SIZE_CM            5.0f    /* Nua canh khoi xop 10x10cm       */

/* Goc lap cam bien trong he than xe (duong = CCW / ben trai mui xe) */
#define TOF_ANGLE_LEFT_DEG              45.0f
#define TOF_ANGLE_FRONT_DEG             0.0f
#define TOF_ANGLE_RIGHT_DEG             (-45.0f)

/* Khoang cach tu tam truc banh xe den mat cam bien ToF (cm) */
#define TOF_OFFSET_LEFT_CM              4.5f
#define TOF_OFFSET_FRONT_CM             5.0f
#define TOF_OFFSET_RIGHT_CM             4.5f

/* Tham so loc nhieu & gom cum phat hien vat can dong */
#define OBS_STATIC_HIT_TOL_CM           2.6f    /* Loai diem cham trung tuong/coc tinh */
#define OBS_STATIC_CENTER_TOL_CM        3.2f    /* Loai tam uoc luong nam trong coc tinh */
#define OBS_STATIC_FILTER_TOL_CM        OBS_STATIC_CENTER_TOL_CM
#define OBS_CLUSTER_RADIUS_CM           8.0f    /* Ban kinh gom cum 3 khung lien       */
#define OBS_CONFIRM_FRAMES              3U      /* So khung moi lien tiep xac nhan     */
#define OBS_DEDUP_RADIUS_CM             11.0f   /* Tranh nap trung 1 vat can           */
#define OBS_MAX_DYNAMIC                 4U      /* Toi da 4 vat can dong               */
#define OBS_DYN_INFLATE_CM              6.75f   /* Noi rong C-Space cho coc dong       */
#define TOF_STALE_TIMEOUT_MS            120U    /* Qua 120ms khong co mau moi -> vo hieu */

/* ============================================================================
 * 4. THAM SO BO LOC EKF 3 TRANG THAI [x, y, theta]
 * ============================================================================ */
#define EKF_ALPHA_SS                    0.0025f /* Phuong sai quang duong theo ds^2   */
#define EKF_ALPHA_ST                    0.0200f /* Nhieu quang duong khi xoay         */
#define EKF_ALPHA_TS                    0.0004f /* Troi goc encoder theo quang duong  */
#define EKF_ALPHA_TT                    0.0150f /* Sai so goc encoder theo dtheta^2   */
#define EKF_Q_FLOOR_POS                 1e-5f   /* San phuong sai vi tri (cm^2)       */
#define EKF_Q_FLOOR_THETA               1e-6f   /* San phuong sai goc (rad^2)         */
#define EKF_R_IMU_YAW_RAD2              0.0004f /* Do tin cay IMU yaw (~1.15 deg std) */
#define EKF_SLIP_OMEGA_DIFF_RADS        0.45f   /* Nguong phat hien truot banh        */
#define EKF_SLIP_Q_SCALE                16.0f   /* Tang tin cay Gyro khi truot banh   */
#define EKF_IMU_FROZEN_TIMEOUT_S        0.24f   /* Bo qua IMU neu dong bang > 0.24s   */
#define EKF_ZUPT_GZ_THRESH_RADS         0.035f  /* Nguong dung yen ZUPT (~2 deg/s)    */

/* ============================================================================
 * 5. AN TOAN PIN & PHAT HIEN KET XE
 * ============================================================================ */
#define ROBOT_VBAT_CRITICAL_V           6.60f   /* Ngat khan cap bao ve LiPo 2S       */
#define ROBOT_VBAT_MIN_VALID_V          4.50f   /* < 4.5V = chay nguon USB debug      */
#define ROBOT_STALL_TIMEOUT_MS          1800U   /* Ket banh > 1.8s -> xu ly an toan   */
#define ROBOT_MAX_RECOVERIES_PER_LAP    2U      /* Gioi han so lan lui tranh ket      */

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_H */
