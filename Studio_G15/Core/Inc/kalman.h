/*
 * Kalman.h
 *
 *  Created on: May 24, 2024
 *      Author: napat
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

#include "main.h"

typedef struct {
    float x_k1_k1; // Previous state
    float P_k1_k1; // Error covariance of the previous state
    float Q;       // Process noise covariance
    float R;       // Measurement noise covariance
    float Kg;      // Kalman gain
    float P_k_k1;  // Predicted error covariance
    float kalman_adc_old; // Previous filtered value
} KalmanState;

void kalman_filter_init(KalmanState *state, float Q, float R);
double kalman_filter(KalmanState *state, double Accel);

#endif /* INC_KALMAN_H_ */
