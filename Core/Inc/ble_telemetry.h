#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * ============================================================================
 *  GIAO THỨC (text, mỗi lệnh kết thúc bằng \n hoặc \r hoặc \r\n)
 *  Không phân biệt hoa/thường. Dấu phân cách: ':'  ','  hoặc khoảng trắng.
 *
 *  Lệnh (Android -> STM32)              Ý nghĩa
 *  ---------------------------------    -----------------------------------------
 *  F | F,<ms>                           Tiến thẳng      (v = +SPD.v, w = 0)
 *  B | B,<ms>                           Lùi             (v = -SPD.v, w = 0)
 *  L | L,<ms>                           Vừa tiến vừa rẽ trái  (v = SPD.v, w = +SPD.w)
 *  R | R,<ms>                           Vừa tiến vừa rẽ phải  (v = SPD.v, w = -SPD.w)
 *  S | STOP                             Dừng khẩn cấp
 *  DRV:<v>,<w>[,<ms>]                   Lái tự do: v [mm/s], w [mrad/s], w>0 = rẽ trái
 *                                       (xoay tại chỗ: DRV:0,800)
 *  SPD:<v>,<w>                          Đặt tốc độ mặc định cho F/B/L/R
 *  OBS:<zone>,<pos>                     Nạp vật cản (cũ: SET:OBS,<zone>,<pos>)
 *  RUN:<1|2>                            1 = lái tay, 2 = tự hành (cũ: CMD:RUN,<n>)
 *  VSEG:<0..5>,<v_cms>                  Hiệu chỉnh v_target [10..30] cm/s (0=tất cả, 1..5=Ô 1..5)
 *  CFG:GET | CFG:SAVE | CFG:RESET       Đọc toàn bộ thông số / Lưu Flash / Khôi phục mặc định
 *  CFG:<KEY>,<VAL>                      Chỉnh live thông số (V1..V5,VALL,VDET,VMAX,ACC,KP,KI,WD,WB,PPR,TOF,SCANX,LD,YAWINV)
 *  ZERO                                 Reset gốc toạ độ + IMU (cũ: CMD:ZERO)
 *  TEL | TEL:<ms>                       TEL = gửi 1 gói ngay; TEL:<ms> = chu kỳ, 0 = tắt
 *  WDG:<ms>                             Watchdog lái tay: quá <ms> không có lệnh -> dừng, 0 = tắt
 *  PING                                 Kiểm tra kết nối
 *
 *  <ms> (tuỳ chọn) = thời gian chạy rồi tự dừng; bỏ qua hoặc 0 = chạy tới khi có lệnh khác.
 *
 *  Phản hồi (STM32 -> Android)
 *  ---------------------------------
 *  $ACK,<LỆNH>[,tham số đã áp dụng]     Đã nhận & chấp nhận. VD: $ACK,DRV,200,0,500
 *  $ERR,<LỆNH>,<ARG|RANGE|REJECT|UNKNOWN|NOHANDLER|TOOLONG>
 *  $EVT,<READY|DRV_DONE|WDG_STOP>       Sự kiện
 *  $TEL,mode,vbat,x,y,heading,dl,df,dr,vright_exp,vleft_exp,vright_real,vleft_real
 *                                       Motor speeds are in rad/s.
 * ============================================================================
 */

/* ============================ Cấu hình ============================ */
#define BLE_RX_CHUNK_SIZE        64      /* buffer HAL nhận mỗi lần (ReceiveToIdle) */
#define BLE_RX_RING_SIZE         256     /* PHẢI là luỹ thừa của 2 */
#define BLE_TX_RING_SIZE         512     /* PHẢI là luỹ thừa của 2 */
#define BLE_TX_CHUNK_SIZE        64      /* số byte tối đa mỗi lần Transmit_IT */
#define BLE_TX_LINE_MAX          128
#define BLE_LINE_MAX             64
#define BLE_TOKEN_MAX            6
#define BLE_LINE_TIMEOUT_MS      50      /* gặp khoảng lặng này mà chưa có \n thì coi như hết lệnh */

#define BLE_V_MAX_MMPS           500     /* giới hạn vận tốc dài  [mm/s]   */
#define BLE_W_MAX_MRADPS         3000    /* giới hạn vận tốc góc  [mrad/s] */
#define BLE_V_DEFAULT_MMPS       200
#define BLE_W_DEFAULT_MRADPS     800
#define BLE_DRV_MAX_DURATION_MS  10000

#define BLE_TEL_MIN_PERIOD_MS    100
#define BLE_TEL_DEFAULT_PERIOD_MS 100   /* 0 = mặc định tắt */
#define BLE_TX_TELEM_BACKLOG     64      /* hàng đợi TX đang có >N byte thì bỏ qua telemetry */
#define BLE_WDG_DEFAULT_MS       0       /* 0 = mặc định tắt (Kai Morich không gửi liên tục) */

/* ============================ Kiểu dữ liệu ============================ */
typedef struct
{
    uint8_t  current_mode;   /* 1-Manual ; 2-Autonomous */
    float    v_battery;
    float    x_pos;
    float    y_pos;
    float    heading;
    uint16_t dist_left;
    uint16_t dist_front;
    uint16_t dist_right;
    float    vright_expected;
    float    vleft_expected;
    float    vright_reality;
    float    vleft_reality;
} RobotTelemetry_t;

/*
 * Các callback được gọi từ BLE_Telemetry_Process() => chạy ở main loop,
 * KHÔNG phải trong ngắt, nên có thể gọi thoải mái hàm điều khiển motor.
 * Trả về true = chấp nhận (sẽ gửi $ACK), false = từ chối (sẽ gửi $ERR,...,REJECT).
 */
typedef bool (*BLE_DriveHandler_t)(int16_t v_mmps, int16_t w_mradps, uint16_t duration_ms);
typedef bool (*BLE_ObstacleHandler_t)(uint8_t zone, uint8_t position);
typedef bool (*BLE_RunModeHandler_t)(uint8_t run_id);
typedef void (*BLE_ZeroCalibHandler_t)(void);
typedef void (*BLE_StopHandler_t)(void);
/* Hieu chinh v_target (seg = 0: tat ca 5 doan, 1..5: O 1..O 5; v_cms trong [10..30]) */
typedef bool (*BLE_VTargetHandler_t)(uint8_t seg_1based, int16_t v_cms);
/* Chinh live & luu Flash thong so he thong */
typedef bool (*BLE_ConfigSetHandler_t)(const char *key, float value);
typedef bool (*BLE_ConfigActionHandler_t)(const char *action);

typedef struct
{
    BLE_DriveHandler_t        on_drive;
    BLE_ObstacleHandler_t     on_set_obstacle;
    BLE_RunModeHandler_t      on_set_mode;
    BLE_ZeroCalibHandler_t    on_zero;
    BLE_StopHandler_t         on_stop;
    BLE_VTargetHandler_t      on_set_vtarget;
    BLE_ConfigSetHandler_t    on_cfg_set;
    BLE_ConfigActionHandler_t on_cfg_action;
} BLE_Callbacks_t;

/* ============================ API ============================ */
void BLE_Telemetry_Init(UART_HandleTypeDef *huart, const BLE_Callbacks_t *callbacks);
void BLE_Telemetry_StartReceive_IT(void);

/* Gọi liên tục trong app_loop(): parse lệnh, watchdog, đẩy dữ liệu TX */
void BLE_Telemetry_Process(void);

/* Gọi từ HAL callback (ngắt) */
void BLE_Telemetry_RxEventCallback(uint16_t size);
void BLE_Telemetry_ErrorCallback(void);

/* Telemetry: ReportDue() trả true khi tới chu kỳ -> app điền struct rồi SendReport() */
bool BLE_Telemetry_ReportDue(void);
bool BLE_Telemetry_SendReport(const RobotTelemetry_t *data);
bool BLE_Telemetry_SendBytes(const uint8_t *data, uint16_t length);

/* Gửi 1 dòng tuỳ ý (tự thêm \r\n). CHỈ gọi từ main context, không gọi trong ngắt. */
bool BLE_Telemetry_SendLine(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif
