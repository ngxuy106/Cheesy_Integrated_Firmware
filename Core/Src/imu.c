#include "imu.h"
#include <math.h>

#if (IMU_DLPF_CFG < 1) || (IMU_DLPF_CFG > 6)
#error "IMU_DLPF_CFG phai trong khoang 1..6 (DLPF bat) de sample rate goc = 1 kHz"
#endif

/* ===================== Thanh ghi ===================== */
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

/* ===================== DWT: đo thời gian chính xác ===================== */
static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { }
}

/* ===================== I2C helper ===================== */
static HAL_StatusTypeDef wr8(IMU_t *m, uint8_t reg, uint8_t v)
{
    return HAL_I2C_Mem_Write(m->hi2c, m->addr, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, IMU_I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef rd8(IMU_t *m, uint8_t reg, uint8_t *v)
{
    return HAL_I2C_Mem_Read(m->hi2c, m->addr, reg, I2C_MEMADD_SIZE_8BIT, v, 1, IMU_I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef rdN(IMU_t *m, uint8_t reg, uint8_t *buf, uint16_t n)
{
    return HAL_I2C_Mem_Read(m->hi2c, m->addr, reg, I2C_MEMADD_SIZE_8BIT, buf, n, IMU_I2C_TIMEOUT_MS);
}

#define CHK(x)  do { if ((x) != HAL_OK) return IMU_ERR_I2C; } while (0)

static inline int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}

static uint8_t id_is_known(uint8_t id)
{
    /* 0x68 = MPU6050 chuẩn (cũng là giá trị trả về khi AD0 = 1).
     * Các module clone thường trả 0x70..0x73 / 0x98 nhưng vẫn tương thích thanh ghi. */
    return (id == 0x68 || id == 0x69 || (id >= 0x70 && id <= 0x73) || id == 0x98);
}

/* ===================== Cấu hình chip ===================== */
static IMU_Status configure(IMU_t *m)
{
    uint8_t id, rb;

    CHK(wr8(m, REG_PWR_MGMT_1, 0x80));          /* device reset */
    HAL_Delay(100);
    CHK(wr8(m, REG_PWR_MGMT_1, 0x01));          /* thoát sleep, clock = PLL gyro X */
    HAL_Delay(50);

    CHK(rd8(m, REG_WHO_AM_I, &id));
    if (!id_is_known(id)) return IMU_ERR_ID;

    CHK(wr8(m, REG_SMPLRT_DIV,  (uint8_t)(1000 / IMU_SAMPLE_RATE_HZ - 1)));
    CHK(wr8(m, REG_CONFIG,      IMU_DLPF_CFG));
    CHK(wr8(m, REG_GYRO_CONFIG, (uint8_t)(IMU_GYRO_FS_SEL << 3)));
    CHK(wr8(m, REG_ACCEL_CONFIG, 0x00));         /* ±2g */

    /* đọc lại để xác nhận */
    CHK(rd8(m, REG_GYRO_CONFIG, &rb));
    if (rb != (uint8_t)(IMU_GYRO_FS_SEL << 3)) return IMU_ERR_CONFIG;
    CHK(rd8(m, REG_CONFIG, &rb));
    if ((rb & 0x07) != IMU_DLPF_CFG) return IMU_ERR_CONFIG;
    return IMU_OK;
}

IMU_Status IMU_Init(IMU_t *m, I2C_HandleTypeDef *hi2c, uint8_t addr_8bit,
                    GPIO_TypeDef *scl_port, uint16_t scl_pin,
                    GPIO_TypeDef *sda_port, uint16_t sda_pin)
{
    if (m == NULL || hi2c == NULL) return IMU_ERR_PARAM;

    m->hi2c = hi2c;
    m->addr = addr_8bit;
    m->scl_port = scl_port; m->scl_pin = scl_pin;
    m->sda_port = sda_port; m->sda_pin = sda_pin;

    m->ax = m->ay = m->az = 0;
    m->gx = m->gy = m->gz = 0;
    m->temp_c = 0;
    m->yaw = 0;
    for (int i = 0; i < 3; i++) { m->raw_g[i] = 0; m->gyro_off[i] = 0; }
    m->prev_gz = 0;
    m->has_last = 0;
    m->yaw_invert = 0;
    m->err_count = 0;
    m->recover_count = 0;
    m->initialized = 0;

    dwt_init();

    m->last_error = configure(m);
    if (m->last_error != IMU_OK) {
        /* thử hồi phục bus một lần rồi cấu hình lại */
        IMU_RecoverBus(m);
        m->last_error = configure(m);
    }
    m->initialized = (m->last_error == IMU_OK);
    return m->last_error;
}

/* ===================== Hiệu chuẩn ===================== */
IMU_Status IMU_CalibrateGyro(IMU_t *m, uint16_t samples)
{
    if (!m->initialized || samples == 0) return IMU_ERR_PARAM;

    int32_t sum[3] = {0, 0, 0};
    int16_t mn[3] = { 32767,  32767,  32767};
    int16_t mx[3] = {-32768, -32768, -32768};
    uint8_t b[6];
    uint32_t period_ms = 1000U / IMU_SAMPLE_RATE_HZ;
    if (period_ms == 0) period_ms = 1;

    HAL_Delay(200);   /* chờ gyro ổn định sau cấu hình */

    for (uint16_t i = 0; i < samples; i++) {
        if (rdN(m, REG_GYRO_XOUT_H, b, 6) != HAL_OK) {
            m->last_error = IMU_ERR_I2C;
            return IMU_ERR_I2C;
        }
        for (int k = 0; k < 3; k++) {
            int16_t v = be16(&b[2 * k]);
            sum[k] += v;
            if (v < mn[k]) mn[k] = v;
            if (v > mx[k]) mx[k] = v;
        }
        HAL_Delay(period_ms);
    }

    for (int k = 0; k < 3; k++) {
        if ((mx[k] - mn[k]) > IMU_CAL_MAX_RANGE_RAW) {
            m->last_error = IMU_ERR_MOVING;
            return IMU_ERR_MOVING;       /* giữ nguyên offset cũ */
        }
    }
    for (int k = 0; k < 3; k++) m->gyro_off[k] = (float)sum[k] / (float)samples;

    m->yaw = 0;
    m->prev_gz = 0;
    m->has_last = 0;
    m->last_error = IMU_OK;
    return IMU_OK;
}

/* ===================== Hồi phục bus I2C ===================== */
void IMU_RecoverBus(IMU_t *m)
{
    dwt_init();
    HAL_I2C_DeInit(m->hi2c);

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    g.Pin = m->scl_pin;  HAL_GPIO_Init(m->scl_port, &g);
    g.Pin = m->sda_pin;  HAL_GPIO_Init(m->sda_port, &g);

    HAL_GPIO_WritePin(m->scl_port, m->scl_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(m->sda_port, m->sda_pin, GPIO_PIN_SET);
    delay_us(10);

    /* Slave đang giữ SDA thấp -> nhấp tối đa 9 xung SCL để nó nhả bus */
    for (int i = 0; i < 9; i++) {
        if (HAL_GPIO_ReadPin(m->sda_port, m->sda_pin) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(m->scl_port, m->scl_pin, GPIO_PIN_RESET);
        delay_us(10);
        HAL_GPIO_WritePin(m->scl_port, m->scl_pin, GPIO_PIN_SET);
        delay_us(10);
    }

    /* Tạo điều kiện STOP: SDA thấp -> cao khi SCL cao */
    HAL_GPIO_WritePin(m->sda_port, m->sda_pin, GPIO_PIN_RESET);
    delay_us(10);
    HAL_GPIO_WritePin(m->scl_port, m->scl_pin, GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(m->sda_port, m->sda_pin, GPIO_PIN_SET);
    delay_us(10);

    HAL_GPIO_DeInit(m->scl_port, m->scl_pin);
    HAL_GPIO_DeInit(m->sda_port, m->sda_pin);

    HAL_I2C_Init(m->hi2c);      /* MspInit trả chân về AF open-drain */

    m->recover_count++;
    m->has_last = 0;            /* bỏ qua dt của khoảng gián đoạn */
}

/* ===================== Đọc & tích phân ===================== */
IMU_Status IMU_Update(IMU_t *m)
{
    uint8_t b[14];

    if (!m->initialized) return IMU_ERR_PARAM;

    if (rdN(m, REG_ACCEL_XOUT_H, b, 14) != HAL_OK) {
        m->has_last = 0;
        if (++m->err_count >= IMU_ERR_RECOVER_THRESH) {
            IMU_RecoverBus(m);
            m->err_count = 0;
            /* chip có thể đã bị reset do sụt áp -> cấu hình lại (giữ nguyên offset) */
            if (configure(m) != IMU_OK) {
                m->last_error = IMU_ERR_I2C;
                return IMU_ERR_I2C;
            }
        }
        m->last_error = IMU_ERR_I2C;
        return IMU_ERR_I2C;
    }
    m->err_count = 0;

    /* Thứ tự: AX AY AZ TEMP GX GY GZ (mỗi giá trị 2 byte, MSB trước) */
    int16_t rax = be16(&b[0]);
    int16_t ray = be16(&b[2]);
    int16_t raz = be16(&b[4]);
    int16_t rt  = be16(&b[6]);
    int16_t rgx = be16(&b[8]);
    int16_t rgy = be16(&b[10]);
    int16_t rgz = be16(&b[12]);

    m->ax = rax / IMU_ACCEL_SCALE;
    m->ay = ray / IMU_ACCEL_SCALE;
    m->az = raz / IMU_ACCEL_SCALE;
    m->temp_c = rt / 340.0f + 36.53f;

    m->raw_g[0] = rgx; m->raw_g[1] = rgy; m->raw_g[2] = rgz;
    m->gx = (rgx - m->gyro_off[0]) / IMU_GYRO_SCALE;
    m->gy = (rgy - m->gyro_off[1]) / IMU_GYRO_SCALE;
    m->gz = (rgz - m->gyro_off[2]) / IMU_GYRO_SCALE;
    if (m->yaw_invert) m->gz = -m->gz;

    /* dt thực tế (DWT), không giả định chu kỳ cố định */
    uint32_t now = DWT->CYCCNT;
    float dt = (float)(uint32_t)(now - m->last_cycle) / (float)SystemCoreClock;
    m->last_cycle = now;

    float gz_db = (fabsf(m->gz) < IMU_GYRO_DEADBAND_DPS) ? 0.0f : m->gz;

    if (m->has_last && dt > 0.0f && dt <= IMU_MAX_DT_S) {
        m->yaw += 0.5f * (m->prev_gz + gz_db) * dt;       /* hình thang */
        if (m->yaw >= 180.0f)      m->yaw -= 360.0f;
        else if (m->yaw < -180.0f) m->yaw += 360.0f;
    }
    m->prev_gz = gz_db;
    m->has_last = 1;

    m->last_error = IMU_OK;
    return IMU_OK;
}

/* Bám theo trôi offset khi xe đứng yên */
void IMU_TrackBias(IMU_t *m, uint8_t still)
{
    const float alpha = 0.002f;
    if (!still || !m->initialized) return;
    if (fabsf(m->gx) > 1.0f || fabsf(m->gy) > 1.0f || fabsf(m->gz) > 1.0f) return;   /* đang xoay thật */
    for (int k = 0; k < 3; k++) {
        m->gyro_off[k] += alpha * (m->raw_g[k] - m->gyro_off[k]);
    }
}

/* ===================== Tiện ích yaw ===================== */
float IMU_GetYaw(const IMU_t *m) 
{ 
    return m->yaw; 
}

void IMU_ResetYaw(IMU_t *m)
{
    m->yaw = 0;
    m->prev_gz = 0;
    m->has_last = 0;
}

void IMU_SetYaw(IMU_t *m, float yaw_deg)
{
    while (yaw_deg >= 180.0f) yaw_deg -= 360.0f;
    while (yaw_deg < -180.0f) yaw_deg += 360.0f;
    m->yaw = yaw_deg;
}

float IMU_AngleDiff(float target, float current)
{
    float d = target - current;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
