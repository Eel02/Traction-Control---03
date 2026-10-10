#include "pi_controller.h"

void PI_Init(PI_Controller_t *pi, float kp, float ki, float dt, float limit) {
    pi->kp = kp;
    pi->ki = ki;
    pi->dt = dt;
    pi->limit_max = limit;
    pi->limit_min = -limit;
    pi->integral_error = 0.0f;
}

float PI_Compute(PI_Controller_t *pi, float setpoint, float measure, uint8_t hold_integral) {
    float error = setpoint - measure;
    float p_action = pi->kp * error;
    
    //Calcolo per valutare la saturazione
    float current_i_action = pi->ki * pi->integral_error;
    float output_provv = p_action + current_i_action;
    
    uint8_t is_sat_high = (output_provv >= pi->limit_max);
    uint8_t is_sat_low = (output_provv <= pi->limit_min);
    
    //Clamping solo se richiesto
    if (hold_integral == 0) {
        if (!(is_sat_high && error > 0.0f) && !(is_sat_low && error < 0.0f)) {
            pi->integral_error += error * pi->dt;
        }
    }

    //Calcolo finale
    float i_action = pi->ki * pi->integral_error;
    float output = p_action + i_action;

    //Saturazione fisica dell'uscita
    if (output > pi->limit_max) {
        output = pi->limit_max;
    } else if (output < pi->limit_min) {
        output = pi->limit_min;
    }

    return output;
}

void PI_Reset(PI_Controller_t *pi) {
    pi->integral_error = 0.0f;
}
