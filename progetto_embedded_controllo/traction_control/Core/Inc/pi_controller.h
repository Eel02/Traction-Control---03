#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

#include <stdint.h>

/* Struttura del controllore PI */
typedef struct {
    float kp;
    float ki;
    float dt;
    float limit_max;
    float limit_min;
    float integral_error;
} PI_Controller_t;

/* Inizializzazione del controllore */
void PI_Init(PI_Controller_t *pi, float kp, float ki, float dt, float limit);

/* Calcolo dell'azione di controllo. 
   L'argomento 'hold_integral' (1 o 0) permette di bloccare l'integrazione */
float PI_Compute(PI_Controller_t *pi, float setpoint, float measure, uint8_t hold_integral);

/* Reset dell'accumulatore integrale */
void PI_Reset(PI_Controller_t *pi);

#endif
