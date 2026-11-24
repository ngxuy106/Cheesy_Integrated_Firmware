#ifndef BATTERY_H
#define BATTERY_H

#include "main.h"
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>

//Battery caculation formula: V_adc_stm32 = Vpin * (5.1k/(5.1k + 10k))

extern volatile float input_voltage;
typedef struct
{
    ADC_HandleTypeDef *hadc;
    GPIO_TypeDef *led_port;
    uint16_t led_pin;
    float vref;
    float r_top_k; //10k ohms
    float r_bottom_k; //5.1k ohms
    float v_min;
    float v_max;
}Battery_Config_t;


//Init
void Battery_Init(Battery_Config_t *config);

// Read ADC and Battery's Voltage
uint16_t Battery_ReadRaw(uint8_t samples);
float Battery_ReadVoltage(uint8_t samples);
float Battery_GetPercentage(void);

// Non-blocking function for alerting battery percentage
void Battery_Update_Indicator(void);


#endif