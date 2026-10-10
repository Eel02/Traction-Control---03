/*
* amt102.c
*
*  Created on: Jul 9, 2026
*      Author: soria
*/
#include "amt102.h"
#include <math.h>
void AMT102_Init(AMT102_Encoder_t* enc) {
   HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);
   enc->absolute_angle = 0.0f;
   enc->velocity_rad_s = 0.0f;
   uint16_t initial_counter = __HAL_TIM_GET_COUNTER(enc->htim);
   enc->last_counter = initial_counter;
}
void AMT102_Update(AMT102_Encoder_t* enc, float dt_seconds) {
   uint16_t current_counter = __HAL_TIM_GET_COUNTER(enc->htim);
   int16_t delta_ticks = (int16_t)(current_counter - enc->last_counter);
   //Aggiorna lo stato per il ciclo successivo
   enc->last_counter = current_counter;
   //Calcolo del delta (angolo) e della velocità grezza in rad/s
   float delta_angle = ((float)delta_ticks / enc->ppr_x4) * (2.0f * (float)M_PI);
   float raw_vel_rad_s = delta_angle / dt_seconds;
   //Filtro EMA sulla velocità
   float alpha = 0.3f;
   enc->velocity_rad_s = (alpha * raw_vel_rad_s) + ((1.0f - alpha) * enc->velocity_rad_s);
   //Aggiornamento posizione assoluta
   enc->absolute_angle += delta_angle;
}
