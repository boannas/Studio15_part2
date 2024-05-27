// Include Library here !
#include <BaseSystem.h>
#include "PS2.h"
#include "main.h"
#include "Motor.h"
#include "ModBusRTU.h"
#include "Encoder.h"
#include "PID_controller.h"
#include  "Trapezoidal.h"

// Import variable from other .c file
extern PID_struct PID_pos;
extern PID_struct PID_velo;
extern BaseStruct base;
extern AMT_Encoder AMT;
extern MOTOR MT;
extern u16u8_t registerFrame[200];
extern Trap_Traj Traj;
extern float temp_pos;
// Define variable inside library
int num[10]= {48,49,50,51,52,53,54,55,56,57};
int count = 0;

// Function Prototypes
void PS2_init(PS2_typedef* PS2);
void PS2X_Reader();
void handle_shelve_mode();
void handle_PIDPos_adjustment();

//-------------------------------------------Function Code-------------------------------------------------------//

extern PS2_typedef ps2;

void PS2_init(PS2_typedef* PS2)
{
    PS2->ps2RX[10] = 0;
    PS2->digit= 0;
    PS2->gain[3] = 0;
    PS2->ps2Y = 0;
    PS2->ps2YPos = 0;
    PS2->floor[5] = 0;
    PS2->r[6] = 0;
    PS2->l[6] = 0;
    PS2->pwmOut = 0;
    PS2->mode = 0;
    PS2->stop = 0;
    PS2->counts = 0;
    PS2->PIDPos = 0;
    PS2->on = 0;
}

void PS2X_Reader()
{
	// Keys Map
	static uint32_t timestamp = 0;
	if (timestamp < HAL_GetTick())
	{
		timestamp = HAL_GetTick() + 25;
		ps2.ps2RX[0] = 0;
	}

	// Clear previous state
	for (int i = 0; i < 6; i++)
	{
		ps2.l[i] = 0;
		ps2.r[i] = 0;
	}

	// Set current state
	if (ps2.ps2RX[0] >= 65 && ps2.ps2RX[0] <= 70) {
		ps2.l[ps2.ps2RX[0] - 65] = 1;
	}
	if (ps2.ps2RX[0] >= 73 && ps2.ps2RX[0] <= 78) {
		ps2.r[ps2.ps2RX[0] - 73] = 1;
	}

    if (ps2.ps2RX[0] == 69) 		// Press L4 to switch to use Joy stick
    {
        ps2.mode = 1;
        ps2.on = 0;
    }
    if (ps2.ps2RX[0] == 70)		// Press L5 to switch to use Button
    {
        ps2.mode = 2;
        PID_velo.out = 0;
        ps2.on = 1;
        ps2.PIDPos = AMT.Linear_Position;
    }

    // Mode Joy stick
    if (ps2.mode == 1)
    {
        static uint32_t debounce_time = 0;
        uint32_t current_time = HAL_GetTick();
        if (current_time > debounce_time)
        {
            if (ps2.l[0] == 1)
            {
                // Increase ps2.PIDPos
            	ps2.pwmOut = 350;
				debounce_time = current_time + 250; // Debounce delay of 250ms
            }
            else if (ps2.l[1] == 1)
            {
                // Decrease ps2.PIDPos
            	ps2.pwmOut = 0;
				debounce_time = current_time + 250; // Debounce delay of 250ms
            }
            else{ps2.pwmOut = 200;}
        }


    }
    // Mode Button
    else if (ps2.mode == 2)
    {
        ps2.on = 1;
        ps2.ps2Y = 0;


        // Handle specific conditions
        if (ps2.ps2RX[0] == 67) {
            ps2.l[2] = 1;
        }
        if (ps2.ps2RX[0] == 68) {
            ps2.l[3] = 1;
        }

        handle_shelve_mode();
        handle_PIDPos_adjustment();
    }
}

void handle_shelve_mode()
{
    static uint32_t debounce_time = 0;
    uint32_t current_time = HAL_GetTick();
    if (base.ShelveMode == 1)
    {
        if (ps2.ps2RX[0] == 76 && current_time > debounce_time)
        {
            // Press Select to save shelve
            base.Shelve[count] = AMT.Linear_Position;
            count += 1;
            debounce_time = current_time + 250; // Debounce delay of 250ms
        }
        else if (ps2.ps2RX[0] == 73 && current_time > debounce_time)
        {
            // Press Triangle to delete old array
            base.Shelve[count - 1] = 0;
            count -= 1;
            debounce_time = current_time + 250; // Debounce delay of 250ms
        }
        else if (ps2.ps2RX[0] == 72 && base.ShelveMode == 1)
        {
            // Press Start to finish set shelves and send data to basesystem
            base.ShelveMode = 0;
            Traj.currentPosition = AMT.Linear_Position;
            temp_pos = AMT.Linear_Position;

        }
    }
}

void handle_PIDPos_adjustment()
{
    static uint32_t debounce_time = 0;
    uint32_t current_time = HAL_GetTick();

    if (current_time > debounce_time)
    {
        if (ps2.l[2] == 1)
        {
            // Increase ps2.PIDPos
            ps2.PIDPos += 0.5;
            debounce_time = current_time + 250; // Debounce delay of 250ms
        }
        else if (ps2.l[3] == 1)
        {
            // Decrease ps2.PIDPos
            ps2.PIDPos -= 0.5;
            debounce_time = current_time + 250; // Debounce delay of 250ms
        }
    }
}

//void PS2X_Reader() {
//    static uint32_t debounce_time = 0; // For debouncing button presses
//    uint32_t current_time = HAL_GetTick();
//
//    // Handle Mode Switches
//    if (ps2.ps2RX[0] == 69) { // Press L4 to switch to Joy stick mode
//        ps2.mode = 1;
//        ps2.on = 0;
//        memset(ps2.l, 0, sizeof(ps2.l));
//        memset(ps2.r, 0, sizeof(ps2.r));
//    } else if (ps2.ps2RX[0] == 70) { // Press L5 to switch to Button mode
//        ps2.mode = 2;
//        PID_velo.out = 0;
//        ps2.on = 1;
//        ps2.PIDPos = AMT.Linear_Position;
//        memset(ps2.l, 0, sizeof(ps2.l));
//        memset(ps2.r, 0, sizeof(ps2.r));
//    }
//
//    if (ps2.mode == 1) { // Joystick Mode
//        // Check for button debouncing in joystick mode
//
//        // Handle button states based on received RX values
//        for (int i = 65; i <= 70; i++) {
//            if (ps2.ps2RX[0] == i) ps2.l[i - 65] = 1;
//        }
//        for (int i = 73; i <= 78; i++) {
//            if (ps2.ps2RX[0] == i) ps2.r[i - 73] = 1;
//        }
//
//        if (current_time > debounce_time) {
//            if (ps2.l[0] == 1) {  // Debounce for button L1
//                ps2.pwmOut = 350;
//                debounce_time = current_time + 250; // Debounce delay of 250ms
//            }
//            else if (ps2.l[1] == 1) {  // Debounce for button L2
//                ps2.pwmOut = 0;
//                debounce_time = current_time + 250; // Debounce delay of 250ms
//            }
//            else{ps2.pwmOut = 200;}
//        }
//        // Additional joystick handling can be added here if needed
//    } else if (ps2.mode == 2) { // Button Mode
//        ps2.on = 1;
//        ps2.ps2Y = 0;
//
//        // Handle button states based on received RX values
//        for (int i = 65; i <= 70; i++) {
//            if (ps2.ps2RX[0] == i) ps2.l[i - 65] = 1;
//        }
//        for (int i = 73; i <= 78; i++) {
//            if (ps2.ps2RX[0] == i) ps2.r[i - 73] = 1;
//        }
//
//        handle_shelve_mode();
//        handle_PIDPos_adjustment();
//    }
//}



