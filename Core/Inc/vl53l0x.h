#ifndef VL53L0X_H
#define VL53L0X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* kéo theo stm32f4xx_hal.h */
#include <stdint.h>

/* Địa chỉ mặc định (dạng 8-bit cho HAL = 0x29 << 1) */
#define VL53L0X_DEFAULT_ADDR      0x52
#define VL53L0X_MODEL_ID          0xEE
#define VL53L0X_OUT_OF_RANGE      8190U     /* giá trị trả về khi không có vật / lỗi */

typedef enum {
    VL53L0X_OK = 0,
    VL53L0X_ERR_I2C,
    VL53L0X_ERR_TIMEOUT,
    VL53L0X_ERR_ID,        /* không đọc được Model ID = 0xEE */
    VL53L0X_ERR_SPAD,
    VL53L0X_ERR_CALIB,
    VL53L0X_ERR_BUDGET,
    VL53L0X_ERR_PARAM
} VL53L0X_Status;

typedef struct {
    /* ---- cấu hình do người dùng điền (qua VL53L0X_Create) ---- */
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef      *xshut_port;
    uint16_t           xshut_pin;
    uint8_t            i2c_addr;        /* địa chỉ 8-bit muốn gán (0x54, 0x56, ...) */

    /* ---- trạng thái nội bộ / kết quả ---- */
    uint8_t            stop_variable;
    uint32_t           timing_budget_us;
    uint8_t            continuous;      /* 1 nếu đang chạy continuous */
    uint8_t            initialized;
    VL53L0X_Status     last_error;

    uint16_t           distance_mm;     /* kết quả đo gần nhất */
    uint8_t            range_status;    /* mã trạng thái thô (11 = hợp lệ) */
    uint8_t            valid;           /* 1 = phép đo tin cậy */
} VL53L0X_Device;

/* Gán thông số phần cứng cho 1 cảm biến */
void VL53L0X_Create(VL53L0X_Device *dev, I2C_HandleTypeDef *hi2c,
                    GPIO_TypeDef *xshut_port, uint16_t xshut_pin,
                    uint8_t new_addr_8bit);

/* Reset bằng XSHUT, đổi địa chỉ từng con, khởi tạo + hiệu chuẩn.
 * Trả về VL53L0X_OK nếu tất cả OK; nếu lỗi, dev[i].last_error cho biết con nào hỏng. */
VL53L0X_Status VL53L0X_InitAll(VL53L0X_Device *devs, uint8_t count);

/* Đo 1 lần (blocking, ~30 ms). Trả về mm hoặc VL53L0X_OUT_OF_RANGE */
uint16_t VL53L0X_ReadSingle(VL53L0X_Device *dev);

/* Chế độ liên tục: period_ms = 0 -> back-to-back, >0 -> đo theo chu kỳ */
VL53L0X_Status VL53L0X_StartContinuous(VL53L0X_Device *dev, uint32_t period_ms);
VL53L0X_Status VL53L0X_StopContinuous(VL53L0X_Device *dev);

/* Non-blocking: gọi liên tục trong vòng lặp. Trả 1 nếu vừa có dữ liệu mới
 * (đã cập nhật dev->distance_mm, dev->valid) */
uint8_t VL53L0X_Update(VL53L0X_Device *dev);

/* Cập nhật cả mảng; trả về số cảm biến có dữ liệu mới */
uint8_t VL53L0X_UpdateAll(VL53L0X_Device *devs, uint8_t count);

/* Tùy chỉnh: timing budget (us). 20000 = nhanh, 33000 = mặc định, 200000 = chính xác cao */
VL53L0X_Status VL53L0X_SetTimingBudget(VL53L0X_Device *dev, uint32_t budget_us);
uint32_t       VL53L0X_GetTimingBudget(VL53L0X_Device *dev);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_H */
