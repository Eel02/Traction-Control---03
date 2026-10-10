/* bts7960.c */
#include "bts7960.h"

void BTS7960_Init(BTS7960_Motor_t* motor) {
    HAL_TIM_PWM_Start(motor->htim, motor->pwm_channel);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, 0);
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
}

void BTS7960_SetDutyCycle(BTS7960_Motor_t* motor, float duty_cycle_percent) {
    if (duty_cycle_percent > 100.0f) duty_cycle_percent = 100.0f;
    if (duty_cycle_percent < -100.0f) duty_cycle_percent = -100.0f;

    uint32_t ccr_value = (uint32_t)((fabsf(duty_cycle_percent) / 100.0f) * motor->max_pwm_value);

    if (duty_cycle_percent > 0.0f) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, ccr_value);
    } else if (duty_cycle_percent < 0.0f) {
        HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, ccr_value);
    } else {
        BTS7960_Stop(motor);
    }
}

void BTS7960_Stop(BTS7960_Motor_t* motor) {
    __HAL_TIM_SET_COMPARE(motor->htim, motor->pwm_channel, 0);
}
