/*
 * Encoder_AMT.c
 *
 *  Created on: May 3, 2024
 *      Author: napat
 */

// Include Library here !
#include "Encoder.h"
#include "main.h"
#include "Kalman.h"
#include "PID_controller.h"

// Import variable from other .c file
extern PID_struct PID_velo;
extern KalmanState filtered_velo;
extern KalmanState filtered_accel;

// Define variable inside library
int32_t diffPosition;
uint32_t cnt_per_rev = 8192;
float R_pulley = 12.73;
float diffTime;
float diffTimeAcc;
const float cnt_per_rev_inv = 1.0f / 8192.0 ;
const float one_million_inv = 1e-6f;


//-------------------------------------------Function Code-------------------------------------------------------//


void AMT_encoder_init(AMT_Encoder *AMT_data,TIM_HandleTypeDef *Encoder_timer)
{
	AMT_data->EncoderTIM = Encoder_timer;
	HAL_TIM_Encoder_Start(AMT_data->EncoderTIM, TIM_CHANNEL_ALL);
	AMT_data->Position[QEI_NOW] = 0.0;
	AMT_data->Position[QEI_PREV] = 0.0;
}

void AMT_encoder_update(AMT_Encoder *AMT_data, TIM_HandleTypeDef *Encoder_timer, uint64_t current_time)
{
	float pulley_cir = 2.0 * (22.0/7.0) * R_pulley;				// 2 * pi * r
    // Collect data
    AMT_data->TimeStamp[QEI_NOW] = current_time;
    AMT_data->Position[QEI_NOW] = __HAL_TIM_GET_COUNTER(Encoder_timer);

    // Position 1 turn calculation
    AMT_data->QEIPostion_1turn = AMT_data->Position[QEI_NOW] % cnt_per_rev;

    // Calculate dx
    int32_t diffPosition = AMT_data->Position[QEI_NOW] - AMT_data->Position[QEI_PREV];

    // Handle wrap-around
    if (diffPosition > 32767)
        diffPosition -= 65535;
    if (diffPosition < -32767)
        diffPosition += 65535;

    // Calculate different time
    uint64_t diffTime = AMT_data->TimeStamp[QEI_NOW] - AMT_data->TimeStamp[QEI_PREV];

    // Calculate angular velocity
    float time_seconds = diffTime * one_million_inv;
    AMT_data->Angular_Velocity = ((diffPosition * 60.0f) * cnt_per_rev_inv / time_seconds);

    // Calculate linear position and velocity
    float position_change_mm = (diffPosition * pulley_cir) * cnt_per_rev_inv;
    AMT_data->Linear_Position += position_change_mm;
    AMT_data->Linear_Velocity = kalman_filter(&filtered_velo,(AMT_data->Angular_Velocity / 60.0f * pulley_cir));
    AMT_data->Linear_Velo[QEI_NOW] = AMT_data->Linear_Velocity;

    // Calculate linear Acceleration
    static uint64_t accel_timestamp = 0;
	AMT_data->Accel_TimeStamp[QEI_NOW] = current_time;
	if(AMT_data->Accel_TimeStamp[QEI_NOW] >= accel_timestamp)
	{
		accel_timestamp = AMT_data->Accel_TimeStamp[QEI_NOW] + 1e4;//us
		diffTimeAcc = (AMT_data->Accel_TimeStamp[QEI_NOW] - AMT_data->Accel_TimeStamp[QEI_PREV]) * 1e-6;
		double accel = (AMT_data->Linear_Velo[QEI_NOW] - AMT_data->Linear_Velo[QEI_PREV]) / diffTimeAcc;
		AMT_data->Linear_Acceleration = kalman_filter(&filtered_accel, accel);
		AMT_data->Linear_Velo[QEI_PREV] = AMT_data->Linear_Velo[QEI_NOW];
		AMT_data->Accel_TimeStamp[QEI_PREV] = AMT_data->Accel_TimeStamp[QEI_NOW];
	}

    // Store value for next loop
    AMT_data->Position[QEI_PREV] = AMT_data->Position[QEI_NOW];
    AMT_data->TimeStamp[QEI_PREV] = AMT_data->TimeStamp[QEI_NOW];
}


void AMT_encoder_reset(AMT_Encoder *AMT_data)
{
	// Set home position at desire position
	AMT_data->Linear_Position = 600;

}

