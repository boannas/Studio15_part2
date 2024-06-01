/*
 * BaseSystem.c
 *
 *  Created on: May 12, 2024
 *      Author: porpo
 */

// Include Library here !
#include "BaseSystem.h"
#include "main.h"
#include "Encoder.h"
#include "Trapezoidal.h"
#include "stm32g4xx_hal.h"
#include "PID_controller.h"
#include "PS2.h"
#include "math.h"

// Import variable from other .c file
extern AMT_Encoder AMT;
extern Trap_Traj Traj;
extern float elapsedTime;
extern float x_pos;
extern PID_struct PID_velo, PID_pos;
extern PS2_typedef ps2;

int fuCount = 0;
float temp_pos;
int temp_cnt = 0;
int temp_home = 0;
float trajInput;
int StartDelay = 0;

typedef enum {
    STATE_CASE_4,
    STATE_DELAY_AFTER_4,
    STATE_CASE_8,
    STATE_DELAY_AFTER_8
} InternalStateType;

InternalStateType internalState = STATE_CASE_4;
uint32_t delayStartTime = 0;
uint32_t pushDelay = 0;
uint32_t initDelay = 0;
uint32_t vacuumPlace = 0;

//-------------------------------------------Function Code-------------------------------------------------------//

void easyCase(){
	base.Base_case = registerFrame[0x01].U16;
	base.Vacuum_case = registerFrame[0x02].U16;
	base.Gripper_case = registerFrame[0x03].U16;
}

void Heartbeat(){
	registerFrame[0x00].U16 = 22881;
}

void Routine(){
	if(registerFrame[0x00].U16 == 18537){
		registerFrame[0x04].U16 = base.ReedStatus;   							//Gripper status 0b0010 = 0000 0000 0000 0010
		registerFrame[0x10].U16 = base.BaseStatus;								//Z-axis status 0010 = 1
		registerFrame[0x11].U16 = AMT.Linear_Position				*10;		//Z-axis position
		registerFrame[0x12].U16 = (int)(AMT.Linear_Velocity)		*10;		//Z-axis speed
		registerFrame[0x13].U16 = (int)(AMT.Linear_Acceleration)	*10;		//Z-axis acceleration
		registerFrame[0x40].U16 = (int)(x_pos)						*10;		//X-axis position
	}
}

void Vacuum(){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, registerFrame[0x02].U16 == 0b0001 ? SET : RESET);
}

void GripperMovement(){
    base.Gripper = registerFrame[0x03].U16;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, registerFrame[0x03].U16 == 0b0001 ? RESET : SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, registerFrame[0x03].U16 == 0b0001 ? SET : RESET);
}


void SetShelves(){
	base.ShelveMode = 1;
	PS2X_Reader();
	if(ps2.ps2RX[0] == 74){
		ps2.stop = 1;
	}
	if (ps2.stop == 1 && ps2.ps2RX[0] == 75){
		ps2.stop = 0;
		base.MotorHome = 150;
	}
		  else if(ps2.stop == 0){
			  PS2X_Reader();
	  	  }
	if (base.ShelveMode == 0)
	{
		registerFrame[0x23].U16 = base.Shelve[0] *10; 	//Position Shelve 1
		registerFrame[0x24].U16 = base.Shelve[1] *10;
		registerFrame[0x25].U16 = base.Shelve[2] *10;
		registerFrame[0x26].U16 = base.Shelve[3] *10;
		registerFrame[0x27].U16 = base.Shelve[4] *10;

		//finish
		base.BaseStatus = 0;
		registerFrame[0x01].U16 = 0;
		registerFrame[0x10].U16 = base.BaseStatus;
		base.Base_case = 0;
	}
}

void RunPoint(){
	base.GoalPoint = (registerFrame[0x30].U16)/10; //Get Goal point from BaseSytem(Point Mode) that we pick/write After pressing Run Button
	elapsedTime += 0.0002;
	Traject(&Traj, temp_pos, base.GoalPoint);
	PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition);
	base.MotorHome = PID_velo.out;
	// Error must less than 0.1 mm
	if(fabs(AMT.Linear_Position - base.GoalPoint) <= 0.1){
		elapsedTime = 0;
		Traj.currentPosition = base.GoalPoint;
		temp_pos = base.GoalPoint;
		base.BaseStatus = 0;
		registerFrame[0x01].U16 = 0;
		registerFrame[0x10].U16 = base.BaseStatus;
	}
}

void SetHome() {
	float targetPosition = (temp_home == 0) ? 612 : 600;
		elapsedTime += 0.0002;
		Traject(&Traj, temp_pos, targetPosition); // Update trajectory
		PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition); // Apply PID control
		base.MotorHome = PID_velo.out;
	// Top photo limit was triggered
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
        AMT_encoder_reset(&AMT); // Reset encoder to zero or the desired home position
        temp_home = 1;
        elapsedTime = 0;
        Traj.currentPosition = 600;
        base.BaseStatus = 0; // Update the base status
        registerFrame[0x01].U16 = 0;
        registerFrame[0x10].U16 = base.BaseStatus;
    }
}

void RunJog() {
    // Define Pick shelf
    base.Pick[4] = registerFrame[0x21].U16 % 10;
    base.Pick[3] = ((registerFrame[0x21].U16 - base.Pick[4]) % 100) / 10;
    base.Pick[2] = ((registerFrame[0x21].U16 % 1000) - ((base.Pick[3] * 10) + base.Pick[4])) / 100;
    base.Pick[1] = ((registerFrame[0x21].U16 % 10000) - (((base.Pick[2] * 100) + (base.Pick[3] * 10) + base.Pick[4]))) / 1000;
    base.Pick[0] = (registerFrame[0x21].U16 - ((base.Pick[1] * 1000 + base.Pick[2] * 100 + base.Pick[3] * 10 + base.Pick[4]))) / 10000;

    // Define Place shelf
    base.Place[4] = registerFrame[0x22].U16 % 10;
    base.Place[3] = ((registerFrame[0x22].U16 - base.Place[4]) % 100) / 10;
    base.Place[2] = ((registerFrame[0x22].U16 % 1000) - ((base.Place[3] * 10) + base.Place[4])) / 100;
    base.Place[1] = ((registerFrame[0x22].U16 % 10000) - (((base.Place[2] * 100) + (base.Place[3] * 10) + base.Place[4]))) / 1000;
    base.Place[0] = (registerFrame[0x22].U16 - ((base.Place[1] * 1000 + base.Place[2] * 100 + base.Place[3] * 10 + base.Place[4]))) / 10000;

    // Manage the jog operation using a state machine
	if(base.Vacuum == 0){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET);			// Vacuum off
	}
	if(base.Vacuum == 1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);				// Vacuum on
	}
    switch (internalState) {
        case STATE_CASE_4:

            if (fuCount == 5) {
                base.BaseStatus = 0;
                registerFrame[0x01].U16 = base.BaseStatus;
                registerFrame[0x10].U16 = 0;
                base.MotorHome = 150;
                internalState = STATE_DELAY_AFTER_4;
                delayStartTime = HAL_GetTick(); 				// Record the start time for delay
                fuCount = 0;
                base.Vacuum = 0;
                StartDelay = 0;
                break;
            }

            // Calculate the target position for picking
            float pickTarget = base.Shelve[base.Pick[fuCount] - 1] + 5	;
            elapsedTime += 0.0002;
            Traject(&Traj, temp_pos, pickTarget); // Update trajectory
            PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition); // Apply PID control
            base.MotorHome = PID_velo.out;
            // Check if the position is close enough to the target

            if(StartDelay == 0){StartDelay = HAL_GetTick();}

            if (HAL_GetTick() - StartDelay >= 2000) // Delay of 1 second
			{
				if (fabs(AMT.Linear_Position - pickTarget) < 2.0) {
					//Vacuum on
					base.Vacuum = 1;
					//Forward
					if(temp_cnt != 1){
						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
					}
					if (temp_cnt == 0 && base.ReedStatus == 1) {
						if (initDelay == 0){
							pushDelay = HAL_GetTick();
							initDelay = 1;
						}
						if (HAL_GetTick() - pushDelay >= 250){
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);			// Backward
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
							temp_cnt = 1;
						}
					} else if (temp_cnt == 1 && base.ReedStatus == 2) {
						elapsedTime = 0;
						temp_pos = AMT.Linear_Position;
						base.runJogMode = 1;
						Traj.currentPosition = pickTarget;
						internalState = STATE_CASE_8; // Move to next state
						initDelay = 0;
						temp_cnt = 0;
					}
				}
			}
            break;

        case STATE_DELAY_AFTER_4:
//        	a = 0;
            if (HAL_GetTick() - delayStartTime >= 1000) { // Delay of 1 second
                internalState = STATE_CASE_4; // Return to pick state after delay
            }
            break;

        case STATE_CASE_8:
            // Calculate the target position for placing
            float placeTarget = base.Shelve[base.Place[fuCount] - 1] + 15;
            elapsedTime += 0.0002;
            Traject(&Traj, temp_pos, placeTarget); // Update trajectory
            PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition); // Apply PID control
            base.MotorHome = PID_velo.out;
            if(vacuumPlace == 0){
            	base.Vacuum = 1;
            }
            else{base.Vacuum = 0;}
            // Check if the position is close enough to the target
            if (fabs(AMT.Linear_Position - placeTarget) < 2) {
            	//Forward
            	if(temp_cnt != 1){
            		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);
            		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
            	}
                if (temp_cnt == 0 && base.ReedStatus == 1) {
                	base.Vacuum = 0;
                	vacuumPlace = 1;
                    if (initDelay == 0){
                    	pushDelay = HAL_GetTick();
                    	initDelay = 1;
                    }
					if (HAL_GetTick() - pushDelay >= 250){
	            		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);			// Backward
	            		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
	            		temp_cnt = 1;
					}
                	// Vacuum off
                } else if (temp_cnt == 1 && base.ReedStatus == 2) {
                    elapsedTime = 0;
                    temp_pos = AMT.Linear_Position;
                    base.runJogMode = 0;
                    fuCount++;
                    Traj.currentPosition = placeTarget;
                    internalState = STATE_DELAY_AFTER_8; // Move to next state
                   	temp_cnt = 0;
					initDelay = 0;
					vacuumPlace = 0;
                }
            }
            break;

        case STATE_DELAY_AFTER_8:
            if (HAL_GetTick() - delayStartTime >= 1000) { // Delay of 1 second
                internalState = STATE_CASE_4; // Return to pick state after delay
                pushDelay = HAL_GetTick();
            }
            break;
    }

    // Pick and place 5 times
    if (base.sp == 1) {
        base.BaseStatus = 0;
        registerFrame[0x01].U16 = base.BaseStatus;
        registerFrame[0x10].U16 = 0;
        base.sp = 0;
    }
}


void Holding_position()
{
	PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition);
	base.MotorHome = PID_velo.out;
	temp_pos = AMT.Linear_Position;
}
