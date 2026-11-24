#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* ===================== Cấu hình ===================== */
#define IMU_ADDR_AD0_LOW        (0x68 << 1)   /* AD0 nối GND (mặc định) */
#define IMU_ADDR_AD0_HIGH       (0x69 << 1)   /* AD0 nối 3V3 */

#define IMU_SAMPLE_RATE_HZ      200           /* tần số mẫu của chip (DLPF phải bật: gốc 1 kHz) */
#define IMU_DLPF_CFG            3             /* 3: 44/42 Hz, 4: 21/20 Hz, 2: 94/98 Hz */
#define IMU_GYRO_FS_SEL         1             /* 0:±250, 1:±500, 2:±1000, 3:±2000 dps */
#define IMU_GYRO_SCALE          65.5f         /* LSB/(dps) cho ±500 dps (±250:131, ±1000:32.8, ±2000:16.4) */
#define IMU_ACCEL_SCALE         16384.0f      /* LSB/g cho ±2g */

#define IMU_I2C_TIMEOUT_MS      5U
#define IMU_ERR_RECOVER_THRESH  3U            /* số lần lỗi liên tiếp trước khi hồi phục bus */

#define IMU_CAL_SAMPLES         1000
#define IMU_CAL_MAX_RANGE_RAW   120           /* max-min của gyro khi hiệu chuẩn; vượt = xe đang bị rung/di chuyển */
#define IMU_GYRO_DEADBAND_DPS   0.10f         /* bỏ qua tốc độ góc nhỏ hơn mức này khi tích phân */
#define IMU_MAX_DT_S            0.100f        /* dt lớn hơn mức này coi như bị gián đoạn, không tích phân */

typedef enum {
    IMU_OK = 0,
    IMU_ERR_I2C,
    IMU_ERR_ID,        /* WHO_AM_I không hợp lệ */
    IMU_ERR_CONFIG,    /* ghi cấu hình nhưng đọc lại không khớp */
    IMU_ERR_MOVING,    /* xe di chuyển trong lúc hiệu chuẩn */
    IMU_ERR_PARAM
} IMU_Status;

typedef struct {
    /* phần cứng */
    I2C_HandleTypeDef *hi2c;
    uint8_t            addr;                 /* dạng 8-bit */
    GPIO_TypeDef      *scl_port;  uint16_t scl_pin;   /* dùng cho hồi phục bus */
    GPIO_TypeDef      *sda_port;  uint16_t sda_pin;

    /* dữ liệu đã đổi đơn vị */
    float ax, ay, az;       /* g */
    float gx, gy, gz;       /* độ/giây, đã trừ offset */
    float temp_c;           /* °C */
    float yaw;              /* độ, [-180, 180). Dương = quay ngược chiều kim đồng hồ (nhìn từ trên xuống) */

    /* nội bộ */
    float    raw_g[3];      /* gyro thô (LSB) */
    float    gyro_off[3];   /* offset (LSB) */
    float    prev_gz;
    uint32_t last_cycle;
    uint8_t  has_last;
    uint8_t  yaw_invert;    /* đặt 1 nếu lắp ngược trục Z */
    uint8_t  initialized;

    /* chẩn đoán */
    uint8_t        err_count;
    uint32_t       recover_count;
    IMU_Status     last_error;
} IMU_t;

/* Khởi tạo: reset chip, kiểm tra ID, đánh thức, cấu hình DLPF/dải đo/sample rate */
IMU_Status IMU_Init(IMU_t *imu, I2C_HandleTypeDef *hi2c, uint8_t addr_8bit,
                    GPIO_TypeDef *scl_port, uint16_t scl_pin,
                    GPIO_TypeDef *sda_port, uint16_t sda_pin);

/* Hiệu chuẩn gyro lúc đứng yên (blocking ~ samples / IMU_SAMPLE_RATE_HZ giây) */
IMU_Status IMU_CalibrateGyro(IMU_t *imu, uint16_t samples);

/* Đọc burst 14 byte, cập nhật ax..gz, temp, yaw. Gọi đều đặn >= 100 Hz từ main loop */
IMU_Status IMU_Update(IMU_t *imu);

/* Gọi sau IMU_Update khi xe đang dừng (motor tắt) để offset tự bám theo trôi nhiệt */
void IMU_TrackBias(IMU_t *imu, uint8_t robot_is_still);

/* Yaw */
float IMU_GetYaw(const IMU_t *imu);
void  IMU_ResetYaw(IMU_t *imu);                 /* đặt yaw hiện tại = 0 */
void  IMU_SetYaw(IMU_t *imu, float yaw_deg);
float IMU_AngleDiff(float target_deg, float current_deg);   /* sai số ngắn nhất, [-180, 180] */

/* Hồi phục bus I2C: DeInit -> nhấp 9 xung SCL -> STOP -> Init */
void IMU_RecoverBus(IMU_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
