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

// Import variable from other .c file
extern AMT_Encoder AMT;
extern Trap_Traj Traj;
extern float elapsedTime;
int fuCount = 0;
extern PID_struct PID_velo, PID_pos;
extern PS2_typedef ps2;
float temp_pos;
//-------------------------------------------Function Code-------------------------------------------------------//

//void Reset(){  	Didn't use for now
//	registerFrame[0x01].U16 = 0;
//	registerFrame[0x10].U16 = 0;
//}

void easyCase(){
	base.Base_case = registerFrame[0x01].U16;
	base.Vacuum_case = registerFrame[0x02].U16;
	base.Gripper_case = registerFrame[0x03].U16;
}

void Heartbeat(){
	registerFrame[0x00].U16 = 22881;
}

void Routine(){
	if(registerFrame[0x00].U16 == 18537)
	{
		//Gripper 0x04 not sure!?!?
		registerFrame[0x04].U16 = base.ReedStatus;   					//Gripper status 0b0010 = 0000 0000 0000 0010
		registerFrame[0x10].U16 = base.BaseStatus;							//Z-axis status 0010 = 1
		registerFrame[0x11].U16 = AMT.Linear_Position			*10;	//Z-axis position
		registerFrame[0x12].U16 = fabs(AMT.Linear_Velocity)		*10;	//Z-axis speed
		registerFrame[0x13].U16 = fabs(AMT.Linear_Acceleration)	*10;	//Z-axis acceleration
		registerFrame[0x40].U16 = 4								*10;	//X-axis position
	}
}

void Vacuum(){
	if(registerFrame[0x02].U16 == 0b0000){
		base.Vacuum = 0;			//Vacuum status: Off
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, RESET);			// Vacuum off


	}
	else if(registerFrame[0x02].U16 == 0b0001){
		base.Vacuum = 1;			//Vacuum status: On
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, SET);			// Vacuum on

	}
}

void GripperMovement(){
	if(registerFrame[0x03].U16 == 0b0000){
		base.Gripper = 0;			//Gripper Movement: Backward
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, SET);			// Backward
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
	}
	else if(registerFrame[0x03].U16 == 0b0001){
		base.Gripper = 1;			//Gripper Movement: Forward
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, RESET);			//Forward
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
	}
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

void SetHome(){
	//	registerFrame[0x01].U16 = 2;
	Traj.currentPosition = 600;
	base.MotorHome = 310;		// Set duty cycle to go upward at slowest speed
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)		// Top photo limit was triggered
	{
//		base.MotorHome = 150;		// Set duty cycle to hold position gripper
		AMT_encoder_reset(&AMT);	// Set linear position to ...
		temp_pos = AMT.Linear_Position;

		base.BaseStatus = 0;
		registerFrame[0x01].U16 = 0;
		registerFrame[0x10].U16 = base.BaseStatus;
	}
}

void RunJog(){

	registerFrame[0x10].U16 = 4;

	// Define Pick shelf
	base.Pick[4] = registerFrame[0x21].U16 % 10;
	base.Pick[3] = ((registerFrame[0x21].U16 - base.Pick[4]) % 100)/10;
	base.Pick[2] = ((registerFrame[0x21].U16 %1000) - ((base.Pick[3]*10)+base.Pick[4]))/100;
	base.Pick[1] = ((registerFrame[0x21].U16 %10000)-(((base.Pick[2]*100)+(base.Pick[3]*10)+base.Pick[4])))/1000;
	base.Pick[0] = (registerFrame[0x21].U16-((base.Pick[1]*1000+base.Pick[2]*100+base.Pick[3]*10+base.Pick[4])))/10000;
	// Define Place shelf
	base.Place[4] = registerFrame[0x22].U16 % 10;
	base.Place[3] = ((registerFrame[0x22].U16 - base.Place[4]) % 100)/10;
	base.Place[2] = ((registerFrame[0x22].U16 %1000) - ((base.Place[3]*10)+base.Place[4]))/100;
	base.Place[1] = ((registerFrame[0x22].U16 %10000)-(((base.Place[2]*100)+(base.Place[3]*10)+base.Place[4])))/1000;
	base.Place[0] = (registerFrame[0x22].U16-((base.Place[1]*1000+base.Place[2]*100+base.Place[3]*10+base.Place[4])))/10000;


	// Condition for Pick and Place
	switch(registerFrame[0x10].U16){
		// Pick Case
		case(4):
			elapsedTime += 0.0002;
			Traject(&Traj, AMT.Linear_Position, base.Pick[fuCount]);
			PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition);
			if (fabs(AMT.Linear_Position - base.Pick[fuCount]) < 0.2)
			{
				registerFrame[0x10].U16 = 8;
				elapsedTime = 0;
			}

		// Place Case
		case(8):

			elapsedTime += 0.0002;
			Traject(&Traj, AMT.Linear_Position, base.Place[fuCount]);
			PID_controller_cascade(&PID_pos, &PID_velo, &AMT, Traj.currentPosition);
			if (fabs(AMT.Linear_Position - base.Place[fuCount]) < 0.2)
			{
				base.BaseStatus = 0;
				registerFrame[0x01].U16 = base.BaseStatus;
				registerFrame[0x10].U16 = 0;
				elapsedTime = 0;
//				base.sp = 0;
//				PID_pos.out = 0;
			}

	}



	//pick place 5 time
	if(base.sp == 1){
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
