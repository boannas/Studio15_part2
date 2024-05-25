/*
 * kalman.c
 *
 *  Created on: May 24, 2024
 *      Author: napat
 */
#include "main.h"
#include "Kalman.h"

void kalman_filter_init(KalmanState *state, float Q, float R)
{
	state->Q = Q;
	state->R = R;
	state->x_k1_k1 = 0;
	state->P_k1_k1 = 1;
	state->Kg = 0;
	state->P_k_k1 = 1;
	state->kalman_adc_old = 0;
}

double kalman_filter(KalmanState *state, double Accel ) {
    float Z_k = Accel;
    float x_k_k1 = state->kalman_adc_old;
    state->P_k_k1 = state->P_k1_k1 + state->Q;

    state->Kg = state->P_k_k1 / (state->P_k_k1 + state->R);
    float kalman_adc = x_k_k1 + state->Kg * (Z_k - state->kalman_adc_old);
    state->P_k1_k1 = (1 - state->Kg) * state->P_k_k1;

    state->kalman_adc_old = kalman_adc;
    return kalman_adc;
}
