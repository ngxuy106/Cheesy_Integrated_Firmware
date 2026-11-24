#ifndef INC_MOTOR_API_H_
#define INC_MOTOR_API_H_

#include "main.h"
#include <stdbool.h>

#define MOTOR_SPEED_FILTER_SIZE 2U 


typedef enum
{
    MOTOR_STATUS_OK = 0,
    MOTOR_STATUS_ERROR,
    MOTOR_STATUS_INVALID_PARAM
}Motor_Status_t;

typedef struct 
{
    //Timer Encoder
    TIM_HandleTypeDef *htim_enc;
    bool is_reversed_encoder;

    //Timer PWM
    TIM_HandleTypeDef *htim_pwm;
    uint32_t pwm_channel;

    // H-Bridge TB6612
    GPIO_TypeDef *dir_port_a;
    GPIO_TypeDef *dir_port_b;
    uint16_t dir_pin_a;
    uint16_t dir_pin_b;
    bool is_reversed_dir;

    //HW specs
    float pulses_per_rev;
    float wheel_diameter_m;
    float control_period_s; // 100Hz
}Motor_Config_t;


// PI Controller (volatile Kp/Ki allow live CubeMonitor tuning without -Os caching)
typedef struct 
{
    volatile float Kp;
    volatile float Ki;
    float max_integral;
    float max_output; // MAX ARR
    float max_speed_rads;
}Motor_PID_Params_t;


// Cubemonitor + ISR-shared state (volatile prevents -Os register caching across TIM10 ISR)
typedef struct 
{
    volatile float dq_cmd_rads;
    volatile float dq_state_rads;

    float error;
    float integral;
    float raw_output; // PWM value

    //Slide moving average filter params
    float filter_buf[MOTOR_SPEED_FILTER_SIZE];
    uint8_t filter_idx;
    uint8_t sample_count;

    volatile int32_t total_ticks; // Accumulated signed encoder ticks (100Hz TIM10 ISR)
}Motor_State_t;


//Motor Handler
typedef struct
{
    Motor_Config_t config;
    Motor_PID_Params_t pi;
    Motor_State_t state;
    bool is_initialized;
    bool is_enabled;
}Motor_Handle_t;



/* ================= API Functions ================= */

/**
 * @brief Init motor
 */
Motor_Status_t motor_init(Motor_Handle_t *hmotor, const Motor_Config_t *cfg, const Motor_PID_Params_t *pid_cfg);


/**
 * @brief Enable/Disable Motor
 */
Motor_Status_t motor_enable(Motor_Handle_t *hmotor);
Motor_Status_t motor_disable(Motor_Handle_t *hmotor);


/**
 * @brief Motor Control (Rad/s)
 */
Motor_Status_t motor_SetTargetSpeed_Rads(Motor_Handle_t *hmotor, float dq_cmd_rads);


/**
 * @brief Update PID Loop, called in TIM10 (Ex: 100Hz)
 */
Motor_Status_t motor_update_loop(Motor_Handle_t *hmotor);



void motor_reset_distance(Motor_Handle_t *hmotor);


#endif


