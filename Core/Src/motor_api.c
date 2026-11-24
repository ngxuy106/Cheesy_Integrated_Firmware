#include "motor_api.h"
#include <stddef.h>
#include <string.h>

#define MOTOR_PI_F          3.14159265358979323846f
#define MOTOR_TWO_PI_F      (2.0f * MOTOR_PI_F)
#define MOTOR_ENCODER_MASK  0xFFFFU

/* Private helpers ---------------------------------------------------------- */

static bool motor_config_is_valid(const Motor_Config_t *cfg,
                                  const Motor_PID_Params_t *pid_cfg)
{
    if ((cfg == NULL) || (pid_cfg == NULL))
    {
        return false;
    }

    if ((cfg->htim_enc == NULL) ||
        (cfg->htim_pwm == NULL) ||
        (cfg->pwm_channel == 0U) ||
        (cfg->dir_port_a == NULL) ||
        (cfg->dir_port_b == NULL) ||
        (cfg->dir_pin_a == 0U) ||
        (cfg->dir_pin_b == 0U))
    {
        return false;
    }

    if ((cfg->pulses_per_rev <= 0.0f) ||
        (cfg->control_period_s <= 0.0f))
    {
        return false;
    }

    if ((pid_cfg->max_integral < 0.0f) ||
        (pid_cfg->max_output <= 0.0f) ||
        (pid_cfg->max_speed_rads <= 0.0f))
    {
        return false;
    }

    return true;
}

static void motor_reset_filter(Motor_Handle_t *hmotor)
{
    uint8_t i;

    for (i = 0U; i < MOTOR_SPEED_FILTER_SIZE; i++)
    {
        hmotor->state.filter_buf[i] = 0.0f;
    }

    hmotor->state.filter_idx = 0U;
    hmotor->state.sample_count = 0U;
}

static float motor_filter_speed(Motor_Handle_t *hmotor, float speed_rad_s)
{
    float sum = 0.0f;
    uint8_t i;

    hmotor->state.filter_buf[hmotor->state.filter_idx] = speed_rad_s;
    hmotor->state.filter_idx++;

    if (hmotor->state.filter_idx >= MOTOR_SPEED_FILTER_SIZE)
    {
        hmotor->state.filter_idx = 0U;
    }

    if (hmotor->state.sample_count < MOTOR_SPEED_FILTER_SIZE)
    {
        hmotor->state.sample_count++;
    }

    for (i = 0U; i < hmotor->state.sample_count; i++)
    {
        sum += hmotor->state.filter_buf[i];
    }

    return sum / (float)hmotor->state.sample_count;
}

/* TB6612 behavior retained from the original motor.c:
 * IN1 = IN2 = HIGH and PWM = 0 when the command is zero/stopped.
 * The shared TB6612 STBY pin is managed by the application/robot layer.
 */
static void motor_apply_stop_output(Motor_Handle_t *hmotor)
{
    HAL_GPIO_WritePin(hmotor->config.dir_port_a,
                      hmotor->config.dir_pin_a,
                      GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor->config.dir_port_b,
                      hmotor->config.dir_pin_b,
                      GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(hmotor->config.htim_pwm,
                          hmotor->config.pwm_channel,
                          0U);
}

static void motor_apply_drive_output(Motor_Handle_t *hmotor, float output)
{
    bool forward;
    uint16_t pwm_value;

    if (output >= 0.0f)
    {
        forward = true;
        pwm_value = (uint16_t)output;
    }
    else
    {
        forward = false;
        pwm_value = (uint16_t)(-output);
    }

    /* Reverse the commanded electrical direction if the motor wiring requires it. */
    if (hmotor->config.is_reversed_dir)
    {
        forward = !forward;
    }

    if (forward)
    {
        HAL_GPIO_WritePin(hmotor->config.dir_port_a,
                          hmotor->config.dir_pin_a,
                          GPIO_PIN_SET);
        HAL_GPIO_WritePin(hmotor->config.dir_port_b,
                          hmotor->config.dir_pin_b,
                          GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(hmotor->config.dir_port_a,
                          hmotor->config.dir_pin_a,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hmotor->config.dir_port_b,
                          hmotor->config.dir_pin_b,
                          GPIO_PIN_SET);
    }

    __HAL_TIM_SET_COMPARE(hmotor->config.htim_pwm,
                          hmotor->config.pwm_channel,
                          pwm_value);
}

/* Read the incremental encoder delta since the previous control update.
 * The original implementation used a signed 16-bit delta and reset CNT to 0.
 */
static int16_t motor_read_encoder_delta(Motor_Handle_t *hmotor)
{
    uint32_t counter = __HAL_TIM_GET_COUNTER(hmotor->config.htim_enc);
    int16_t delta = (int16_t)(counter & MOTOR_ENCODER_MASK);

    __HAL_TIM_SET_COUNTER(hmotor->config.htim_enc, 0U);

    if (hmotor->config.is_reversed_encoder)
    {
        delta = (int16_t)(-delta);
    }

    return delta;
}

static void motor_update_measured_speed(Motor_Handle_t *hmotor)
{
    int16_t encoder_delta = motor_read_encoder_delta(hmotor);
    float speed_rad_s;

    hmotor->state.total_ticks += encoder_delta; //test Square-shaped

    /* rad/s = encoder counts per control period * 2*pi / counts per revolution / dt */
    speed_rad_s = ((float)encoder_delta * MOTOR_TWO_PI_F) /
                  (hmotor->config.pulses_per_rev *
                   hmotor->config.control_period_s);

    hmotor->state.dq_state_rads = motor_filter_speed(hmotor, speed_rad_s);
}

/* Public API ---------------------------------------------------------------- */

Motor_Status_t motor_init(Motor_Handle_t *hmotor,
                          const Motor_Config_t *cfg,
                          const Motor_PID_Params_t *pid_cfg)
{
    HAL_StatusTypeDef hal_status;

    if (hmotor == NULL)
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if (!motor_config_is_valid(cfg, pid_cfg))
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    /* Copy configuration before clearing runtime state. */
    hmotor->config = *cfg;
    hmotor->pi = *pid_cfg;
    (void)memset(&hmotor->state, 0, sizeof(hmotor->state));
    hmotor->is_initialized = false;
    hmotor->is_enabled = false;

    /* Put the bridge outputs in a safe zero-command state before starting PWM. */
    motor_apply_stop_output(hmotor);

    hal_status = HAL_TIM_Encoder_Start(hmotor->config.htim_enc,
                                       TIM_CHANNEL_ALL);
    if (hal_status != HAL_OK)
    {
        return MOTOR_STATUS_ERROR;
    }

    hal_status = HAL_TIM_PWM_Start(hmotor->config.htim_pwm,
                                   hmotor->config.pwm_channel);
    if (hal_status != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(hmotor->config.htim_enc,
                                   TIM_CHANNEL_ALL);
        return MOTOR_STATUS_ERROR;
    }

    __HAL_TIM_SET_COUNTER(hmotor->config.htim_enc, 0U);
    motor_reset_filter(hmotor);
    motor_apply_stop_output(hmotor);

    hmotor->is_initialized = true;
    return MOTOR_STATUS_OK;
}

Motor_Status_t motor_enable(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if (!hmotor->is_initialized)
    {
        return MOTOR_STATUS_ERROR;
    }

    hmotor->is_enabled = true;
    hmotor->state.error = 0.0f;
    hmotor->state.integral = 0.0f;
    hmotor->state.raw_output = 0.0f;
    motor_reset_filter(hmotor);
    __HAL_TIM_SET_COUNTER(hmotor->config.htim_enc, 0U);
    motor_apply_stop_output(hmotor);

    return MOTOR_STATUS_OK;
}

Motor_Status_t motor_disable(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if (!hmotor->is_initialized)
    {
        return MOTOR_STATUS_ERROR;
    }

    hmotor->is_enabled = false;
    hmotor->state.dq_cmd_rads = 0.0f;
    hmotor->state.error = 0.0f;
    hmotor->state.integral = 0.0f;
    hmotor->state.raw_output = 0.0f;
    motor_reset_filter(hmotor);
    __HAL_TIM_SET_COUNTER(hmotor->config.htim_enc, 0U);
    motor_apply_stop_output(hmotor);

    return MOTOR_STATUS_OK;
}

Motor_Status_t motor_SetTargetSpeed_Rads(Motor_Handle_t *hmotor,
                                         float dq_cmd_rads)
{
    if (hmotor == NULL)
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if (!hmotor->is_initialized)
    {
        return MOTOR_STATUS_ERROR;
    }

    /* Reject NaN; infinities are clamped by the limit checks below. */
    if (dq_cmd_rads != dq_cmd_rads)
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if (dq_cmd_rads > hmotor->pi.max_speed_rads)
    {
        dq_cmd_rads = hmotor->pi.max_speed_rads;
    }
    else if (dq_cmd_rads < -hmotor->pi.max_speed_rads)
    {
        dq_cmd_rads = -hmotor->pi.max_speed_rads;
    }

    hmotor->state.dq_cmd_rads = dq_cmd_rads;
    return MOTOR_STATUS_OK;
}

Motor_Status_t motor_update_loop(Motor_Handle_t *hmotor)
{
    if (hmotor == NULL)
    {
        return MOTOR_STATUS_INVALID_PARAM;
    }

    if (!hmotor->is_initialized)
    {
        return MOTOR_STATUS_ERROR;
    }

    /* Continue consuming encoder counts even while disabled to avoid stale deltas. */
    motor_update_measured_speed(hmotor);

    if ((!hmotor->is_enabled) || (hmotor->state.dq_cmd_rads == 0.0f))
    {
        hmotor->state.error = 0.0f;
        hmotor->state.integral = 0.0f;
        hmotor->state.raw_output = 0.0f;
        motor_apply_stop_output(hmotor);
        return MOTOR_STATUS_OK;
    }

    /* PI control. Integral accumulation is intentionally per sample to preserve
     * the tuning convention used by the original motor.c implementation.
     */
    hmotor->state.error = hmotor->state.dq_cmd_rads -
                          hmotor->state.dq_state_rads;

    hmotor->state.integral += hmotor->state.error;

    if (hmotor->state.integral > hmotor->pi.max_integral)
    {
        hmotor->state.integral = hmotor->pi.max_integral;
    }
    else if (hmotor->state.integral < -hmotor->pi.max_integral)
    {
        hmotor->state.integral = -hmotor->pi.max_integral;
    }

    hmotor->state.raw_output =
        (hmotor->pi.Kp * hmotor->state.error) +
        (hmotor->pi.Ki * hmotor->state.integral);

    if (hmotor->state.raw_output > hmotor->pi.max_output)
    {
        hmotor->state.raw_output = hmotor->pi.max_output;
    }
    else if (hmotor->state.raw_output < -hmotor->pi.max_output)
    {
        hmotor->state.raw_output = -hmotor->pi.max_output;
    }

    motor_apply_drive_output(hmotor, hmotor->state.raw_output);
    return MOTOR_STATUS_OK;
}



//test square-shaped
void motor_reset_distance(Motor_Handle_t *hmotor) {
    hmotor->state.total_ticks = 0;
}
