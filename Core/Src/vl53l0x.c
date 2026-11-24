#include "vl53l0x.h"

/* ====================== Thanh ghi ====================== */
#define REG_SYSRANGE_START                          0x00
#define REG_SYSTEM_SEQUENCE_CONFIG                  0x01
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD          0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR                  0x0B
#define REG_RESULT_INTERRUPT_STATUS                 0x13
#define REG_RESULT_RANGE_STATUS                     0x14
#define REG_MSRC_CONFIG_CONTROL                     0x60
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_LIMIT 0x44
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51
#define REG_MSRC_CONFIG_TIMEOUT_MACROP              0x46
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6
#define REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD     0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFFSET        0x4F
#define REG_GPIO_HV_MUX_ACTIVE_HIGH                 0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV        0x89
#define REG_I2C_SLAVE_DEVICE_ADDRESS                0x8A
#define REG_IDENTIFICATION_MODEL_ID                 0xC0
#define REG_OSC_CALIBRATE_VAL                       0xF8

#define I2C_TIMEOUT_MS      20U
#define WAIT_TIMEOUT_MS     500U

/* ====================== I2C helper ====================== */
static HAL_StatusTypeDef wr8(VL53L0X_Device *d, uint8_t reg, uint8_t v)
{
    return HAL_I2C_Mem_Write(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &v, 1, I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef rd8(VL53L0X_Device *d, uint8_t reg, uint8_t *v)
{
    return HAL_I2C_Mem_Read(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, v, 1, I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef wr16(VL53L0X_Device *d, uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    return HAL_I2C_Mem_Write(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 2, I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef rd16(VL53L0X_Device *d, uint8_t reg, uint16_t *v)
{
    uint8_t b[2];
    HAL_StatusTypeDef s = HAL_I2C_Mem_Read(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 2, I2C_TIMEOUT_MS);
    if (s == HAL_OK) *v = (uint16_t)((b[0] << 8) | b[1]);
    return s;
}
static HAL_StatusTypeDef wr32(VL53L0X_Device *d, uint8_t reg, uint32_t v)
{
    uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    return HAL_I2C_Mem_Write(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, b, 4, I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef wrMulti(VL53L0X_Device *d, uint8_t reg, uint8_t *src, uint8_t n)
{
    return HAL_I2C_Mem_Write(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, src, n, I2C_TIMEOUT_MS);
}
static HAL_StatusTypeDef rdMulti(VL53L0X_Device *d, uint8_t reg, uint8_t *dst, uint8_t n)
{
    return HAL_I2C_Mem_Read(d->hi2c, d->i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, dst, n, I2C_TIMEOUT_MS);
}

#define CHK(x)  do { if ((x) != HAL_OK) return VL53L0X_ERR_I2C; } while (0)

/* ====================== Chuyển đổi timeout ====================== */
typedef struct {
    uint8_t tcc, msrc, dss, pre_range, final_range;
} StepEnables;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} StepTimeouts;

static uint16_t decodeTimeout(uint16_t reg)
{
    return (uint16_t)((reg & 0x00FF) << ((reg & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint32_t mclks)
{
    uint32_t ls = 0;
    uint16_t ms = 0;
    if (mclks > 0) {
        ls = mclks - 1;
        while ((ls & 0xFFFFFF00UL) > 0) { ls >>= 1; ms++; }
        return (uint16_t)((ms << 8) | (ls & 0xFF));
    }
    return 0;
}

static uint32_t calcMacroPeriod(uint16_t vcsel_pclks)
{
    return ((uint32_t)2304 * vcsel_pclks * 1655 + 500) / 1000;
}

static uint32_t mclksToUs(uint16_t mclks, uint16_t vcsel_pclks)
{
    uint32_t mp = calcMacroPeriod(vcsel_pclks);
    return ((mclks * mp) + (mp / 2)) / 1000;
}

static uint32_t usToMclks(uint32_t us, uint16_t vcsel_pclks)
{
    uint32_t mp = calcMacroPeriod(vcsel_pclks);
    return (((us * 1000) + (mp / 2)) / mp);
}

static VL53L0X_Status getEnables(VL53L0X_Device *d, StepEnables *e)
{
    uint8_t s;
    CHK(rd8(d, REG_SYSTEM_SEQUENCE_CONFIG, &s));
    e->tcc         = (s >> 4) & 1;
    e->dss         = (s >> 3) & 1;
    e->msrc        = (s >> 2) & 1;
    e->pre_range   = (s >> 6) & 1;
    e->final_range = (s >> 7) & 1;
    return VL53L0X_OK;
}

static VL53L0X_Status getTimeouts(VL53L0X_Device *d, const StepEnables *e, StepTimeouts *t)
{
    uint8_t  v8;
    uint16_t v16;

    CHK(rd8(d, REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, &v8));
    t->pre_range_vcsel_period_pclks = (uint16_t)((v8 + 1) << 1);

    CHK(rd8(d, REG_MSRC_CONFIG_TIMEOUT_MACROP, &v8));
    t->msrc_dss_tcc_mclks = (uint16_t)(v8 + 1);
    t->msrc_dss_tcc_us = mclksToUs(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_period_pclks);

    CHK(rd16(d, REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &v16));
    t->pre_range_mclks = decodeTimeout(v16);
    t->pre_range_us = mclksToUs(t->pre_range_mclks, t->pre_range_vcsel_period_pclks);

    CHK(rd8(d, REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, &v8));
    t->final_range_vcsel_period_pclks = (uint16_t)((v8 + 1) << 1);

    CHK(rd16(d, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &v16));
    t->final_range_mclks = decodeTimeout(v16);
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = mclksToUs(t->final_range_mclks, t->final_range_vcsel_period_pclks);
    return VL53L0X_OK;
}

/* Overhead (us) */
#define START_OVERHEAD       1910UL
#define END_OVERHEAD          960UL
#define MSRC_OVERHEAD         660UL
#define TCC_OVERHEAD          590UL
#define DSS_OVERHEAD          690UL
#define PRE_RANGE_OVERHEAD    660UL
#define FINAL_RANGE_OVERHEAD  550UL

static VL53L0X_Status readTimingBudget(VL53L0X_Device *d, uint32_t *budget)
{
    StepEnables e; StepTimeouts t;
    VL53L0X_Status s;
    if ((s = getEnables(d, &e)) != VL53L0X_OK) return s;
    if ((s = getTimeouts(d, &e, &t)) != VL53L0X_OK) return s;

    uint32_t b = START_OVERHEAD + END_OVERHEAD;
    if (e.tcc) b += t.msrc_dss_tcc_us + TCC_OVERHEAD;
    if (e.dss) b += 2 * (t.msrc_dss_tcc_us + DSS_OVERHEAD);
    else if (e.msrc) b += t.msrc_dss_tcc_us + MSRC_OVERHEAD;
    if (e.pre_range)   b += t.pre_range_us + PRE_RANGE_OVERHEAD;
    if (e.final_range) b += t.final_range_us + FINAL_RANGE_OVERHEAD;
    *budget = b;
    return VL53L0X_OK;
}

VL53L0X_Status VL53L0X_SetTimingBudget(VL53L0X_Device *d, uint32_t budget_us)
{
    if (budget_us < 20000UL) return VL53L0X_ERR_PARAM;

    StepEnables e; StepTimeouts t;
    VL53L0X_Status s;
    if ((s = getEnables(d, &e)) != VL53L0X_OK) return s;
    if ((s = getTimeouts(d, &e, &t)) != VL53L0X_OK) return s;

    uint32_t used = START_OVERHEAD + END_OVERHEAD;
    if (e.tcc) used += t.msrc_dss_tcc_us + TCC_OVERHEAD;
    if (e.dss) used += 2 * (t.msrc_dss_tcc_us + DSS_OVERHEAD);
    else if (e.msrc) used += t.msrc_dss_tcc_us + MSRC_OVERHEAD;
    if (e.pre_range) used += t.pre_range_us + PRE_RANGE_OVERHEAD;

    if (e.final_range) {
        used += FINAL_RANGE_OVERHEAD;
        if (used > budget_us) return VL53L0X_ERR_BUDGET;

        uint32_t final_us = budget_us - used;
        uint32_t final_mclks = usToMclks(final_us, t.final_range_vcsel_period_pclks);
        if (e.pre_range) final_mclks += t.pre_range_mclks;
        CHK(wr16(d, REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_mclks)));
        d->timing_budget_us = budget_us;
    }
    return VL53L0X_OK;
}

uint32_t VL53L0X_GetTimingBudget(VL53L0X_Device *d)
{
    return d->timing_budget_us;
}

/* ====================== Khởi tạo nội bộ ====================== */
static VL53L0X_Status waitFlag(VL53L0X_Device *d, uint8_t reg, uint8_t mask, uint8_t want_set)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t v;
    for (;;) {
        CHK(rd8(d, reg, &v));
        if (want_set ? ((v & mask) != 0) : ((v & mask) == 0)) return VL53L0X_OK;
        if ((HAL_GetTick() - t0) > WAIT_TIMEOUT_MS) return VL53L0X_ERR_TIMEOUT;
    }
}

static VL53L0X_Status getSpadInfo(VL53L0X_Device *d, uint8_t *count, uint8_t *is_aperture)
{
    uint8_t tmp;
    VL53L0X_Status s;

    CHK(wr8(d, 0x80, 0x01));
    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, 0x00, 0x00));
    CHK(wr8(d, 0xFF, 0x06));
    CHK(rd8(d, 0x83, &tmp));  CHK(wr8(d, 0x83, tmp | 0x04));
    CHK(wr8(d, 0xFF, 0x07));
    CHK(wr8(d, 0x81, 0x01));
    CHK(wr8(d, 0x80, 0x01));
    CHK(wr8(d, 0x94, 0x6B));
    CHK(wr8(d, 0x83, 0x00));

    /* chờ thanh ghi 0x83 != 0 */
    {
        uint32_t t0 = HAL_GetTick();
        for (;;) {
            CHK(rd8(d, 0x83, &tmp));
            if (tmp != 0x00) break;
            if ((HAL_GetTick() - t0) > WAIT_TIMEOUT_MS) return VL53L0X_ERR_SPAD;
        }
    }
    (void)s;

    CHK(wr8(d, 0x83, 0x01));
    CHK(rd8(d, 0x92, &tmp));
    *count       = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;

    CHK(wr8(d, 0x81, 0x00));
    CHK(wr8(d, 0xFF, 0x06));
    CHK(rd8(d, 0x83, &tmp));  CHK(wr8(d, 0x83, tmp & ~0x04));
    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, 0x00, 0x01));
    CHK(wr8(d, 0xFF, 0x00));
    CHK(wr8(d, 0x80, 0x00));
    return VL53L0X_OK;
}

static VL53L0X_Status singleRefCalibration(VL53L0X_Device *d, uint8_t vhv_init_byte)
{
    VL53L0X_Status s;
    CHK(wr8(d, REG_SYSRANGE_START, 0x01 | vhv_init_byte));
    if ((s = waitFlag(d, REG_RESULT_INTERRUPT_STATUS, 0x07, 1)) != VL53L0X_OK) return s;
    CHK(wr8(d, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));
    CHK(wr8(d, REG_SYSRANGE_START, 0x00));
    return VL53L0X_OK;
}

/* Bộ tuning mặc định của ST (cặp reg, value) */
static const uint8_t tuning_settings[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00}
};

/* Giả định dev->i2c_addr đã đúng và chip đã sẵn sàng */
static VL53L0X_Status initSensor(VL53L0X_Device *d)
{
    VL53L0X_Status s;
    uint8_t v;

    /* Kiểm tra ID */
    CHK(rd8(d, REG_IDENTIFICATION_MODEL_ID, &v));
    if (v != VL53L0X_MODEL_ID) return VL53L0X_ERR_ID;

    /* I/O 2V8 */
    CHK(rd8(d, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, &v));
    CHK(wr8(d, REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, v | 0x01));

    /* I2C standard mode + đọc stop_variable */
    CHK(wr8(d, 0x88, 0x00));
    CHK(wr8(d, 0x80, 0x01));
    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, 0x00, 0x00));
    CHK(rd8(d, 0x91, &d->stop_variable));
    CHK(wr8(d, 0x00, 0x01));
    CHK(wr8(d, 0xFF, 0x00));
    CHK(wr8(d, 0x80, 0x00));

    /* Tắt kiểm tra giới hạn tín hiệu MSRC và pre-range */
    CHK(rd8(d, REG_MSRC_CONFIG_CONTROL, &v));
    CHK(wr8(d, REG_MSRC_CONFIG_CONTROL, v | 0x12));

    /* Giới hạn tốc độ tín hiệu 0.25 MCPS (0.25 * 128 = 32) */
    CHK(wr16(d, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_LIMIT, 32));

    CHK(wr8(d, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF));

    /* ---- SPAD ---- */
    uint8_t spad_count, spad_aperture;
    if ((s = getSpadInfo(d, &spad_count, &spad_aperture)) != VL53L0X_OK) return s;

    uint8_t ref_spad_map[6];
    CHK(rdMulti(d, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6));

    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00));
    CHK(wr8(d, REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C));
    CHK(wr8(d, 0xFF, 0x00));
    CHK(wr8(d, REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4));

    uint8_t first = spad_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count) {
            ref_spad_map[i / 8] &= (uint8_t)~(1U << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            enabled++;
        }
    }
    CHK(wrMulti(d, REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6));

    /* ---- Tuning settings ---- */
    for (uint32_t i = 0; i < (sizeof(tuning_settings) / 2); i++) {
        CHK(wr8(d, tuning_settings[i][0], tuning_settings[i][1]));
    }

    /* Ngắt "new sample ready", mức active-low */
    CHK(wr8(d, REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
    CHK(rd8(d, REG_GPIO_HV_MUX_ACTIVE_HIGH, &v));
    CHK(wr8(d, REG_GPIO_HV_MUX_ACTIVE_HIGH, v & ~0x10));
    CHK(wr8(d, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));

    /* Timing budget */
    uint32_t budget;
    if ((s = readTimingBudget(d, &budget)) != VL53L0X_OK) return s;

    CHK(wr8(d, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8));   /* tắt MSRC và TCC */
    if ((s = VL53L0X_SetTimingBudget(d, budget)) != VL53L0X_OK) return s;

    /* ---- Hiệu chuẩn VHV và phase ---- */
    CHK(wr8(d, REG_SYSTEM_SEQUENCE_CONFIG, 0x01));
    if (singleRefCalibration(d, 0x40) != VL53L0X_OK) return VL53L0X_ERR_CALIB;
    CHK(wr8(d, REG_SYSTEM_SEQUENCE_CONFIG, 0x02));
    if (singleRefCalibration(d, 0x00) != VL53L0X_OK) return VL53L0X_ERR_CALIB;
    CHK(wr8(d, REG_SYSTEM_SEQUENCE_CONFIG, 0xE8));

    d->continuous  = 0;
    d->initialized = 1;
    return VL53L0X_OK;
}

/* ====================== API công khai ====================== */
void VL53L0X_Create(VL53L0X_Device *dev, I2C_HandleTypeDef *hi2c,
                    GPIO_TypeDef *xshut_port, uint16_t xshut_pin,
                    uint8_t new_addr_8bit)
{
    dev->hi2c        = hi2c;
    dev->xshut_port  = xshut_port;
    dev->xshut_pin   = xshut_pin;
    dev->i2c_addr    = new_addr_8bit;
    dev->stop_variable = 0;
    dev->timing_budget_us = 0;
    dev->continuous  = 0;
    dev->initialized = 0;
    dev->last_error  = VL53L0X_OK;
    dev->distance_mm = VL53L0X_OUT_OF_RANGE;
    dev->range_status = 0xFF;
    dev->valid       = 0;
}

VL53L0X_Status VL53L0X_InitAll(VL53L0X_Device *devs, uint8_t count)
{
    VL53L0X_Status result = VL53L0X_OK;

    /* B1: giữ tất cả ở trạng thái reset */
    for (uint8_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(devs[i].xshut_port, devs[i].xshut_pin, GPIO_PIN_RESET);
    }
    HAL_Delay(10);

    /* B2: bật lần lượt từng con, đổi địa chỉ rồi khởi tạo */
    for (uint8_t i = 0; i < count; i++) {
        VL53L0X_Device *d = &devs[i];
        uint8_t target = d->i2c_addr;
        uint8_t v = 0;

        d->initialized = 0;
        HAL_GPIO_WritePin(d->xshut_port, d->xshut_pin, GPIO_PIN_SET);
        HAL_Delay(10);                       /* chờ chip boot (>= 2 ms) */

        /* Nói chuyện ở địa chỉ mặc định */
        d->i2c_addr = VL53L0X_DEFAULT_ADDR;
        if (rd8(d, REG_IDENTIFICATION_MODEL_ID, &v) != HAL_OK || v != VL53L0X_MODEL_ID) {
            d->i2c_addr = target;
            d->last_error = (v != 0 && v != VL53L0X_MODEL_ID) ? VL53L0X_ERR_ID : VL53L0X_ERR_I2C;
            result = d->last_error;
            continue;
        }

        /* Đổi địa chỉ (ghi 7-bit) */
        if (wr8(d, REG_I2C_SLAVE_DEVICE_ADDRESS, (target >> 1) & 0x7F) != HAL_OK) {
            d->i2c_addr = target;
            d->last_error = VL53L0X_ERR_I2C;
            result = d->last_error;
            continue;
        }
        d->i2c_addr = target;

        /* Khởi tạo đầy đủ ở địa chỉ mới */
        d->last_error = initSensor(d);
        if (d->last_error != VL53L0X_OK) result = d->last_error;
    }
    return result;
}

/* Đọc kết quả: 12 byte từ 0x14 -> byte0 = range status, byte10..11 = khoảng cách */
static VL53L0X_Status fetchResult(VL53L0X_Device *d)
{
    uint8_t b[12];
    CHK(rdMulti(d, REG_RESULT_RANGE_STATUS, b, 12));
    uint16_t mm = (uint16_t)((b[10] << 8) | b[11]);
    d->range_status = (b[0] >> 3) & 0x0F;

    if (mm > 20 && mm < VL53L0X_OUT_OF_RANGE) {
        d->distance_mm = mm;
    }

    d->valid        = (d->range_status == 11 && mm > 20 && mm < VL53L0X_OUT_OF_RANGE) ? 1 : 0;
    CHK(wr8(d, REG_SYSTEM_INTERRUPT_CLEAR, 0x01));
    return VL53L0X_OK;
}

static VL53L0X_Status prepareStart(VL53L0X_Device *d)
{
    CHK(wr8(d, 0x80, 0x01));
    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, 0x00, 0x00));
    CHK(wr8(d, 0x91, d->stop_variable));
    CHK(wr8(d, 0x00, 0x01));
    CHK(wr8(d, 0xFF, 0x00));
    CHK(wr8(d, 0x80, 0x00));
    return VL53L0X_OK;
}

uint16_t VL53L0X_ReadSingle(VL53L0X_Device *d)
{
    d->valid = 0;
    d->distance_mm = VL53L0X_OUT_OF_RANGE;
    if (!d->initialized || d->continuous) return VL53L0X_OUT_OF_RANGE;

    if (prepareStart(d) != VL53L0X_OK) { d->last_error = VL53L0X_ERR_I2C; return VL53L0X_OUT_OF_RANGE; }
    if (wr8(d, REG_SYSRANGE_START, 0x01) != HAL_OK) { d->last_error = VL53L0X_ERR_I2C; return VL53L0X_OUT_OF_RANGE; }

    VL53L0X_Status s = waitFlag(d, REG_SYSRANGE_START, 0x01, 0);           /* bit start tự xóa */
    if (s == VL53L0X_OK) s = waitFlag(d, REG_RESULT_INTERRUPT_STATUS, 0x07, 1);
    if (s == VL53L0X_OK) s = fetchResult(d);
    d->last_error = s;
    return (s == VL53L0X_OK) ? d->distance_mm : VL53L0X_OUT_OF_RANGE;
}

VL53L0X_Status VL53L0X_StartContinuous(VL53L0X_Device *d, uint32_t period_ms)
{
    if (!d->initialized) return VL53L0X_ERR_PARAM;
    VL53L0X_Status s = prepareStart(d);
    if (s != VL53L0X_OK) return s;

    if (period_ms != 0) {
        uint16_t osc;
        CHK(rd16(d, REG_OSC_CALIBRATE_VAL, &osc));
        if (osc != 0) period_ms *= osc;
        CHK(wr32(d, REG_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms));
        CHK(wr8(d, REG_SYSRANGE_START, 0x04));   /* timed */
    } else {
        CHK(wr8(d, REG_SYSRANGE_START, 0x02));   /* back-to-back */
    }
    d->continuous = 1;
    return VL53L0X_OK;
}

VL53L0X_Status VL53L0X_StopContinuous(VL53L0X_Device *d)
{
    CHK(wr8(d, REG_SYSRANGE_START, 0x01));
    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, 0x00, 0x00));
    CHK(wr8(d, 0x91, 0x00));
    CHK(wr8(d, 0x00, 0x01));
    CHK(wr8(d, 0xFF, 0x00));
    d->continuous = 0;
    return VL53L0X_OK;
}

uint8_t VL53L0X_Update(VL53L0X_Device *d)
{
    uint8_t st;
    if (!d->initialized || !d->continuous) return 0;
    if (rd8(d, REG_RESULT_INTERRUPT_STATUS, &st) != HAL_OK) {
        d->last_error = VL53L0X_ERR_I2C;
        d->valid = 0;
        return 0;
    }
    if ((st & 0x07) == 0) return 0;              /* chưa có dữ liệu */
    if (fetchResult(d) != VL53L0X_OK) {
        d->last_error = VL53L0X_ERR_I2C;
        d->valid = 0;
        return 0;
    }
    return 1;
}

uint8_t VL53L0X_UpdateAll(VL53L0X_Device *devs, uint8_t count)
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < count; i++) n += VL53L0X_Update(&devs[i]);
    return n;
}
