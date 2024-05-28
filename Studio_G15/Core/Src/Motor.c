/*
 * Motor.c
 *
 *  Created on: May 4, 2024
 *      Author: napat
 */

// Include Library here!
#include "Motor.h"
#include "math.h"

//-------------------------------------------Function Code-------------------------------------------------------//

void MOTOR_init(MOTOR* MT, TIM_HandleTypeDef* htimx, uint16_t timd_chx, uint16_t timp_chx)
{
    if (MT == NULL || htimx == NULL) {
        return;
    }
    MT->htimx = htimx;
    MT->timd_chx = timd_chx;
    MT->timp_chx = timp_chx;

    HAL_TIM_PWM_Start_IT(htimx, timp_chx); // PWM
    HAL_TIM_PWM_Start_IT(htimx, timd_chx); // Direction
}

void MOTOR_set_duty(MOTOR* MT, float percent_duty)
{
    if (MT == NULL || MT->htimx == NULL) {
        return;
    }

    const float max_duty = 42499.0f;
    float scaled_duty = percent_duty * 42.499f;

    if (scaled_duty > max_duty) {
        scaled_duty = max_duty;
    } else if (scaled_duty < -max_duty) {
        scaled_duty = -max_duty;
    }

    if (scaled_duty == 0) {
        __HAL_TIM_SET_COMPARE(MT->htimx, MT->timd_chx, 0);
        __HAL_TIM_SET_COMPARE(MT->htimx, MT->timp_chx, 0);
    } else if (scaled_duty > 0) {
        __HAL_TIM_SET_COMPARE(MT->htimx, MT->timd_chx, 0);
        __HAL_TIM_SET_COMPARE(MT->htimx, MT->timp_chx, (uint32_t)scaled_duty);
    } else {
        __HAL_TIM_SET_COMPARE(MT->htimx, MT->timd_chx, (uint32_t)max_duty);
        __HAL_TIM_SET_COMPARE(MT->htimx, MT->timp_chx, (uint32_t)fabs(scaled_duty));
    }
}
