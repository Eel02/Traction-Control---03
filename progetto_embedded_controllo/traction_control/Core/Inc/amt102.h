#ifndef AMT102_H
#define AMT102_H
#include "main.h"
typedef struct {
   TIM_HandleTypeDef* htim;
   float ppr_x4;
   float velocity_rad_s;
   float absolute_angle;
   uint16_t last_counter;
} AMT102_Encoder_t;
void AMT102_Init(AMT102_Encoder_t* enc);
void AMT102_Update(AMT102_Encoder_t* enc, float dt_seconds);
#endif
