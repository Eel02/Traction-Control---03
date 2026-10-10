/* bts7960.h */
#ifndef INC_BTS7960_H_
#define INC_BTS7960_H_

#include "stm32f4xx_hal.h"
#include <math.h>

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t pwm_channel;

    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;

    uint32_t max_pwm_value;
} BTS7960_Motor_t;

void BTS7960_Init(BTS7960_Motor_t* motor);
void BTS7960_SetDutyCycle(BTS7960_Motor_t* motor, float duty_cycle_percent);
void BTS7960_Stop(BTS7960_Motor_t* motor);

#endif
