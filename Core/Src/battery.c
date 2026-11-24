#include "battery.h"

static Battery_Config_t cfg;
static uint32_t last_adc_time = 0;
static uint32_t last_led_time = 0;
static uint32_t blink_half_period_ms = 1000; // T = 2000ms
volatile float input_voltage = 0.0f;


void Battery_Init(Battery_Config_t *config)
{
    cfg = *config;
}

uint16_t Battery_ReadRaw(uint8_t samples)
{
    if (samples == 0) samples = 1;
    uint32_t sum = 0;
    
    for (uint8_t i = 0; i < samples; i++)
    {
        HAL_ADC_Start(cfg.hadc);
        if (HAL_ADC_PollForConversion(cfg.hadc, 10) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(cfg.hadc);
        }
        HAL_ADC_Stop(cfg.hadc);
    }

    return (uint16_t)(sum / samples);
}

float Battery_ReadVoltage(uint8_t samples)
{
    uint16_t raw = Battery_ReadRaw(samples);
    
    // V_adc (12-bit: 0 - 4095)
    float v_pin = ((float)raw / 4095.0f) * cfg.vref;

    // Vbat = Vpin * (R1 + R2) / R2
    float divider_factor = (cfg.r_top_k + cfg.r_bottom_k) / cfg.r_bottom_k;
    return v_pin * divider_factor;
}

float Battery_GetPercentage(void)
{
    float vbat = Battery_ReadVoltage(8);

    if (vbat <= cfg.v_min) return 0.0f;
    if (vbat >= cfg.v_max) return 100.0f;

    return ((vbat - cfg.v_min) / (cfg.v_max - cfg.v_min)) * 100.0f;
}

void Battery_Update_Indicator(void)
{
    uint32_t now = HAL_GetTick();

    if(now - last_adc_time >= 500)
    {
        last_adc_time = now;
        float percent = Battery_GetPercentage();

        if(percent >= 90.0f)
        {
            blink_half_period_ms = 1000;
        }

        else if(percent >= 30.0f)
        {
            blink_half_period_ms = 500;
        }

        else if(percent < 30.0f)
        {
            blink_half_period_ms = 50;
        }
        // else // heartbeat
        // {
        //     blink_half_period_ms = 5000;
        // }
    }

    if(now - last_led_time >= blink_half_period_ms)
    {
        last_led_time = now;
        HAL_GPIO_TogglePin(cfg.led_port, cfg.led_pin);
    } 
}





