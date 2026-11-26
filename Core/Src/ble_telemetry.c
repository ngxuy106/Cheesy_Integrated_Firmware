#include "ble_telemetry.h"
#include "ram_monitor.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#define RX_MASK  (BLE_RX_RING_SIZE - 1u)
#define TX_MASK  (BLE_TX_RING_SIZE - 1u)

_Static_assert((BLE_RX_RING_SIZE & (BLE_RX_RING_SIZE - 1)) == 0, "BLE_RX_RING_SIZE phai la luy thua cua 2");
_Static_assert((BLE_TX_RING_SIZE & (BLE_TX_RING_SIZE - 1)) == 0, "BLE_TX_RING_SIZE phai la luy thua cua 2");

static UART_HandleTypeDef *ble_uart = NULL;
static BLE_Callbacks_t cbs;

/* ---- RX: ngắt (producer) -> ring -> main loop (consumer) ---- */
static uint8_t           rx_chunk[BLE_RX_CHUNK_SIZE];
static volatile uint8_t  rx_ring[BLE_RX_RING_SIZE];
static volatile uint16_t rx_head = 0;      /* chỉ ngắt ghi  */
static volatile uint16_t rx_tail = 0;      /* chỉ main ghi  */
static volatile uint32_t rx_overflow = 0;

static char     line_buf[BLE_LINE_MAX];
static uint8_t  line_len = 0;
static bool     line_overflow = false;
static uint32_t last_byte_tick = 0;

/* ---- TX: chỉ dùng ở main context (không cần khoá ngắt) ---- */
static uint8_t  tx_ring[BLE_TX_RING_SIZE];
static uint16_t tx_head = 0;
static uint16_t tx_tail = 0;
static uint8_t  tx_chunk[BLE_TX_CHUNK_SIZE];

/* ---- Trạng thái giao thức ---- */
static uint32_t last_rx_tick = 0;
static bool     manual_active = false;
static int      speed_v = BLE_V_DEFAULT_MMPS;
static int      speed_w = BLE_W_DEFAULT_MRADPS;
static uint32_t wdg_ms = BLE_WDG_DEFAULT_MS;
static uint32_t tel_period_ms = BLE_TEL_DEFAULT_PERIOD_MS;
static uint32_t last_tel_tick = 0;
static bool     tel_once = false;

/* ============================ TX ============================ */

static uint16_t tx_used(void)
{
    return (uint16_t)((tx_head - tx_tail) & TX_MASK);
}

static bool tx_enqueue(const char *data, uint16_t len)
{
    uint16_t free_bytes = (uint16_t)(BLE_TX_RING_SIZE - 1u - tx_used());
    if (len > free_bytes) return false;          /* không đủ chỗ -> bỏ cả dòng */
    for (uint16_t i = 0; i < len; i++) {
        tx_ring[tx_head] = (uint8_t)data[i];
        tx_head = (uint16_t)((tx_head + 1u) & TX_MASK);
    }
    return true;
}

/* Đẩy 1 khúc dữ liệu ra UART bằng ngắt. Dùng gState của HAL để biết UART có rảnh không. */
static void tx_pump(void)
{
    if (ble_uart == NULL) return;
    if (ble_uart->gState != HAL_UART_STATE_READY) return;

    uint16_t used = tx_used();
    if (used == 0) return;

    uint16_t n = (used > BLE_TX_CHUNK_SIZE) ? BLE_TX_CHUNK_SIZE : used;
    for (uint16_t i = 0; i < n; i++) {
        tx_chunk[i] = tx_ring[(tx_tail + i) & TX_MASK];
    }
    if (HAL_UART_Transmit_IT(ble_uart, tx_chunk, n) == HAL_OK) {
        tx_tail = (uint16_t)((tx_tail + n) & TX_MASK);
    }
    /* nếu HAL_BUSY: giữ nguyên dữ liệu, lần Process sau thử lại */
}

bool BLE_Telemetry_SendLine(const char *fmt, ...)
{
    char buf[BLE_TX_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 2, fmt, ap);
    va_end(ap);

    if (n <= 0 || n >= (int)sizeof(buf) - 2) return false;
    buf[n++] = '\r';
    buf[n++] = '\n';

    bool ok = tx_enqueue(buf, (uint16_t)n);
    tx_pump();
    return ok;
}

bool BLE_Telemetry_SendBytes(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U) return false;
    bool ok = tx_enqueue((const char *)data, length);
    tx_pump();
    return ok;
}

static void err(const char *cmd, const char *why)
{
    BLE_Telemetry_SendLine("$ERR,%s,%s", cmd, why);
}

/* In số thực dạng fixed-point để không phụ thuộc printf-float của newlib-nano */
static void fmt_fixed(char *dst, size_t size, float v, uint8_t decimals)
{
    int32_t scale = (decimals == 2) ? 100 : 10;
    int32_t q = (int32_t)(v * (float)scale + (v >= 0.0f ? 0.5f : -0.5f));
    uint32_t a = (q < 0) ? (uint32_t)(-q) : (uint32_t)q;
    const char *sign = (q < 0) ? "-" : "";
    if (decimals == 2) {
        snprintf(dst, size, "%s%lu.%02lu", sign, (unsigned long)(a / 100u), (unsigned long)(a % 100u));
    } else {
        snprintf(dst, size, "%s%lu.%lu", sign, (unsigned long)(a / 10u), (unsigned long)(a % 10u));
    }
}

bool BLE_Telemetry_SendReport(const RobotTelemetry_t *data)
{
    if (data == NULL) return false;
    /* Telemetry ưu tiên thấp hơn ACK: hàng đợi đang bận thì bỏ gói này */
    if (tx_used() > BLE_TX_TELEM_BACKLOG) return false;

    char vb[12], x[14], y[14], h[14];
    char vr_exp[14], vl_exp[14], vr_real[14], vl_real[14];
    fmt_fixed(vb, sizeof(vb), data->v_battery, 2);
    fmt_fixed(x,  sizeof(x),  data->x_pos,     1);
    fmt_fixed(y,  sizeof(y),  data->y_pos,     1);
    fmt_fixed(h,  sizeof(h),  data->heading,   1);
    fmt_fixed(vr_exp, sizeof(vr_exp), data->vright_expected, 2);
    fmt_fixed(vl_exp, sizeof(vl_exp), data->vleft_expected, 2);
    fmt_fixed(vr_real, sizeof(vr_real), data->vright_reality, 2);
    fmt_fixed(vl_real, sizeof(vl_real), data->vleft_reality, 2);

    return BLE_Telemetry_SendLine(
        "$TEL,%u,%s,%s,%s,%s,%u,%u,%u,%s,%s,%s,%s",
        (unsigned)data->current_mode, vb, x, y, h,
        (unsigned)data->dist_left,
        (unsigned)data->dist_front,
        (unsigned)data->dist_right,
        vr_exp, vl_exp, vr_real, vl_real);

}

bool BLE_Telemetry_ReportDue(void)
{
    uint32_t now = HAL_GetTick();
    if (tel_once) {
        tel_once = false;
        last_tel_tick = now;
        return true;
    }
    if (tel_period_ms != 0 && (now - last_tel_tick) >= tel_period_ms) {
        last_tel_tick = now;
        return true;
    }
    return false;
}

/* ============================ Parser ============================ */

static void str_upper(char *s)
{
    for (; *s; s++) {
        if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 32);
    }
}

static bool is_delim(char c)
{
    return c == ':' || c == ',' || c == ' ' || c == '\t';
}

static int tokenize(char *s, char *tok[], int max)
{
    int n = 0;
    while (*s && n < max) {
        while (*s && is_delim(*s)) s++;
        if (*s == '\0') break;
        tok[n++] = s;
        while (*s && !is_delim(*s)) s++;
        if (*s) *s++ = '\0';
    }
    return n;
}

static bool parse_int(const char *s, int *out)
{
    char *end;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    *out = (int)v;
    return true;
}

static bool parse_float_simple(const char *s, float *out)
{
    if (s == NULL || *s == '\0' || out == NULL) return false;
    float sign = 1.0f;
    if (*s == '-') {
        sign = -1.0f;
        s++;
    } else if (*s == '+') {
        s++;
    }
    if (*s == '\0') return false;

    float int_part = 0.0f;
    float frac_part = 0.0f;
    float frac_div = 1.0f;
    bool seen_digit = false;
    bool seen_dot = false;

    for (; *s != '\0'; s++) {
        if (*s >= '0' && *s <= '9') {
            seen_digit = true;
            if (!seen_dot) {
                int_part = int_part * 10.0f + (float)(*s - '0');
            } else {
                frac_div *= 10.0f;
                frac_part += (float)(*s - '0') / frac_div;
            }
        } else if (*s == '.' && !seen_dot) {
            seen_dot = true;
        } else {
            return false;
        }
    }
    if (!seen_digit) return false;
    *out = sign * (int_part + frac_part);
    return true;
}

static void do_drive(const char *name, int v, int w, int ms)
{
    if (abs(v) > BLE_V_MAX_MMPS || abs(w) > BLE_W_MAX_MRADPS ||
        ms < 0 || ms > BLE_DRV_MAX_DURATION_MS) {
        err(name, "RANGE");
        return;
    }
    if (cbs.on_drive == NULL) {
        err(name, "NOHANDLER");
        return;
    }
    if (cbs.on_drive((int16_t)v, (int16_t)w, (uint16_t)ms)) {
        manual_active = true;
        BLE_Telemetry_SendLine("$ACK,%s,%d,%d,%d", name, v, w, ms);
    } else {
        err(name, "REJECT");
    }
}

static void do_move(char key, int argc, char **arg)
{
    char name[2] = { key, '\0' };
    int ms = 0;
    if (argc >= 1 && !parse_int(arg[0], &ms)) {
        err(name, "ARG");
        return;
    }
    int v = 0, w = 0;
    switch (key) {
    case 'F': v =  speed_v;                  break;
    case 'B': v = -speed_v;                  break;
    case 'L': v =  speed_v; w =  speed_w;    break;
    case 'R': v =  speed_v; w = -speed_w;    break;
    default: break;
    }
    do_drive(name, v, w, ms);
}

static void do_stop(void)
{
    manual_active = false;
    if (cbs.on_stop) cbs.on_stop();
}

static void handle_line(char *line)
{
    char *tok[BLE_TOKEN_MAX];
    int n = tokenize(line, tok, BLE_TOKEN_MAX);
    if (n == 0) return;

    str_upper(tok[0]);
    int i = 0;
    /* Tương thích lệnh cũ: CMD:STOP, CMD:RUN,2, CMD:ZERO, SET:OBS,4,2 */
    if ((strcmp(tok[0], "CMD") == 0 || strcmp(tok[0], "SET") == 0) && n > 1) {
        i = 1;
        str_upper(tok[1]);
    }
    const char *cmd = tok[i];
    char **arg = &tok[i + 1];
    int argc = n - i - 1;
    int a = 0, b = 0, c = 0;

    if (strcmp(cmd, "PING") == 0) {
        BLE_Telemetry_SendLine("$ACK,PING");
    }
    else if (strcmp(cmd, "STOP") == 0 || strcmp(cmd, "S") == 0) {
        do_stop();
        BLE_Telemetry_SendLine("$ACK,STOP");
    }
    else if (cmd[1] == '\0' && (cmd[0] == 'F' || cmd[0] == 'B' || cmd[0] == 'L' || cmd[0] == 'R')) {
        do_move(cmd[0], argc, arg);
    }
    else if (strcmp(cmd, "DRV") == 0) {
        if (argc < 2 || !parse_int(arg[0], &a) || !parse_int(arg[1], &b) ||
            (argc >= 3 && !parse_int(arg[2], &c))) {
            err("DRV", "ARG");
        } else {
            do_drive("DRV", a, b, c);
        }
    }
    else if (strcmp(cmd, "SPD") == 0) {
        if (argc < 2 || !parse_int(arg[0], &a) || !parse_int(arg[1], &b)) {
            err("SPD", "ARG");
        } else if (a < 1 || a > BLE_V_MAX_MMPS || b < 0 || b > BLE_W_MAX_MRADPS) {
            err("SPD", "RANGE");
        } else {
            speed_v = a;
            speed_w = b;
            BLE_Telemetry_SendLine("$ACK,SPD,%d,%d", a, b);
        }
    }
    else if (strcmp(cmd, "OBS") == 0) {
        if (argc < 2 || !parse_int(arg[0], &a) || !parse_int(arg[1], &b)) {
            err("OBS", "ARG");
        } else if (a < 0 || a > 255 || b < 0 || b > 255) {
            err("OBS", "RANGE");
        } else if (cbs.on_set_obstacle == NULL) {
            err("OBS", "NOHANDLER");
        } else if (cbs.on_set_obstacle((uint8_t)a, (uint8_t)b)) {
            BLE_Telemetry_SendLine("$ACK,OBS,%d,%d", a, b);
        } else {
            err("OBS", "REJECT");
        }
    }
    else if (strcmp(cmd, "RUN") == 0) {
        if (argc < 1 || !parse_int(arg[0], &a)) {
            err("RUN", "ARG");
        } else if (a < 0 || a > 255) {
            err("RUN", "RANGE");
        } else if (cbs.on_set_mode == NULL) {
            err("RUN", "NOHANDLER");
        } else if (cbs.on_set_mode((uint8_t)a)) {
            BLE_Telemetry_SendLine("$ACK,RUN,%d", a);
        } else {
            err("RUN", "REJECT");
        }
    }
    else if (strcmp(cmd, "VSEG") == 0) {
        if (argc < 2 || !parse_int(arg[0], &a) || !parse_int(arg[1], &b)) {
            err("VSEG", "ARG");
        } else if (a < 0 || a > 5 || b < 10 || b > 30) {
            err("VSEG", "RANGE");
        } else if (cbs.on_set_vtarget == NULL) {
            err("VSEG", "NOHANDLER");
        } else if (cbs.on_set_vtarget((uint8_t)a, (int16_t)b)) {
            BLE_Telemetry_SendLine("$ACK,VSEG,%d,%d", a, b);
        } else {
            err("VSEG", "REJECT");
        }
    }
    else if (strcmp(cmd, "CFG") == 0) {
        if (argc == 1) {
            str_upper(arg[0]);
            if (strcmp(arg[0], "GET") != 0 &&
                strcmp(arg[0], "SAVE") != 0 &&
                strcmp(arg[0], "RESET") != 0) {
                err("CFG", "ARG");
            } else if (cbs.on_cfg_action == NULL) {
                err("CFG", "NOHANDLER");
            } else if (cbs.on_cfg_action(arg[0])) {
                if (strcmp(arg[0], "GET") != 0) {
                    BLE_Telemetry_SendLine("$ACK,CFG,%s", arg[0]);
                }
            } else {
                err("CFG", "REJECT");
            }
        } else if (argc >= 2) {
            float fval = 0.0f;
            str_upper(arg[0]);
            if (!parse_float_simple(arg[1], &fval)) {
                err("CFG", "ARG");
            } else if (cbs.on_cfg_set == NULL) {
                err("CFG", "NOHANDLER");
            } else if (cbs.on_cfg_set(arg[0], fval)) {
                char fbuf[16];
                fmt_fixed(fbuf, sizeof(fbuf), fval, 2);
                BLE_Telemetry_SendLine("$ACK,CFG,%s,%s", arg[0], fbuf);
            } else {
                err("CFG", "REJECT");
            }
        } else {
            err("CFG", "ARG");
        }
    }
    else if (strcmp(cmd, "ZERO") == 0) {
        if (cbs.on_zero) cbs.on_zero();
        BLE_Telemetry_SendLine("$ACK,ZERO");
    }
    else if (strcmp(cmd, "TEL") == 0) {
        if (argc == 0) {
            tel_once = true;                 /* câu trả lời chính là gói $TEL */
        } else if (!parse_int(arg[0], &a)) {
            err("TEL", "ARG");
        } else if (a != 0 && (a < BLE_TEL_MIN_PERIOD_MS || a > 60000)) {
            err("TEL", "RANGE");
        } else {
            tel_period_ms = (uint32_t)a;
            last_tel_tick = HAL_GetTick();
            BLE_Telemetry_SendLine("$ACK,TEL,%d", a);
        }
    }
    else if (strcmp(cmd, "WDG") == 0) {
        if (argc < 1 || !parse_int(arg[0], &a)) {
            err("WDG", "ARG");
        } else if (a != 0 && (a < 100 || a > 10000)) {
            err("WDG", "RANGE");
        } else {
            wdg_ms = (uint32_t)a;
            BLE_Telemetry_SendLine("$ACK,WDG,%d", a);
        }
    }
    else {
        err(cmd, "UNKNOWN");
    }
}

static void line_commit(void)
{
    if (line_overflow) {
        err("LINE", "TOOLONG");
    } else if (line_len > 0) {
        line_buf[line_len] = '\0';
        last_rx_tick = HAL_GetTick();
        handle_line(line_buf);
    }
    line_len = 0;
    line_overflow = false;
}

/* ============================ RX / init ============================ */

void BLE_Telemetry_StartReceive_IT(void)
{
    if (ble_uart != NULL) {
        HAL_UARTEx_ReceiveToIdle_IT(ble_uart, rx_chunk, sizeof(rx_chunk));
    }
}

/* Chạy trong ngắt UART: chỉ copy byte vào ring rồi nhận tiếp, không parse. */
void BLE_Telemetry_RxEventCallback(uint16_t size)
{
    if (size > BLE_RX_CHUNK_SIZE) size = BLE_RX_CHUNK_SIZE;

    for (uint16_t i = 0; i < size; i++) {
        uint16_t next = (uint16_t)((rx_head + 1u) & RX_MASK);
        if (next == rx_tail) {           /* ring đầy -> bỏ phần còn lại */
            rx_overflow++;
            break;
        }
        rx_ring[rx_head] = rx_chunk[i];
        rx_head = next;
    }
    BLE_Telemetry_StartReceive_IT();
}

/* Lỗi ORE/FE/NE làm HAL dừng nhận -> bật nhận lại */
void BLE_Telemetry_ErrorCallback(void)
{
    BLE_Telemetry_StartReceive_IT();
}

void BLE_Telemetry_Init(UART_HandleTypeDef *huart, const BLE_Callbacks_t *callbacks)
{
    ble_uart = huart;
    memset(&cbs, 0, sizeof(cbs));
    if (callbacks != NULL) {
        cbs = *callbacks;
    }

    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;
    line_len = 0;
    line_overflow = false;
    manual_active = false;
    speed_v = BLE_V_DEFAULT_MMPS;
    speed_w = BLE_W_DEFAULT_MRADPS;
    wdg_ms = BLE_WDG_DEFAULT_MS;
    tel_period_ms = BLE_TEL_DEFAULT_PERIOD_MS;
    tel_once = false;
    last_rx_tick = last_byte_tick = last_tel_tick = HAL_GetTick();

    BLE_Telemetry_StartReceive_IT();
    BLE_Telemetry_SendLine("$EVT,READY");
}

/* ============================ Process (main loop) ============================ */

void BLE_Telemetry_Process(void)
{
    if (ble_uart == NULL) return;

    /* Tự phục hồi: nếu vì lý do gì đó UART không còn ở trạng thái nhận thì bật lại */
    if (ble_uart->RxState == HAL_UART_STATE_READY) {
        BLE_Telemetry_StartReceive_IT();
    }

    /* 1. Ghép byte thành dòng và thực thi */
    uint32_t now = HAL_GetTick();
    while (rx_tail != rx_head) {
        uint8_t ch = rx_ring[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) & RX_MASK);
        last_byte_tick = now;

        if (RAM_Monitor_ProcessByte(ch)) continue;
        if (ch == '\n' || ch == '\r') {
            line_commit();
        } else if (line_len < BLE_LINE_MAX - 1) {
            line_buf[line_len++] = (char)ch;
        } else {
            line_overflow = true;
        }
    }
    /* Phòng khi app không gửi ký tự xuống dòng */
    if (line_len > 0 && (now - last_byte_tick) >= BLE_LINE_TIMEOUT_MS) {
        line_commit();
    }

    /* 2. Watchdog lái tay */
    if (wdg_ms != 0 && manual_active && (HAL_GetTick() - last_rx_tick) > wdg_ms) {
        do_stop();
        BLE_Telemetry_SendLine("$EVT,WDG_STOP");
    }

    /* 3. Đẩy dữ liệu TX còn đọng */
    tx_pump();
}
