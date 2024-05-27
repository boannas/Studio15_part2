// Include Library here !
#include <BaseSystem.h>
#include "PS2.h"
#include "main.h"
#include "Motor.h"
#include "ModBusRTU.h"
#include "Encoder.h"
#include "PID_controller.h"

// Import variable from other .c file
PID_struct PID_pos;
extern PID_struct PID_velo;
BaseStruct base;
extern AMT_Encoder AMT;
extern MOTOR MT;
extern u16u8_t registerFrame[200];
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
        // Read Ps2 Joy stick in VERTICAL
        if (ps2.ps2RX[0] == 81)
        {
            if (ps2.ps2RX[2] == 83) {
                ps2.digit = 1;
            }
            else if (ps2.ps2RX[3] == 83) {
                ps2.digit = 2;
            }
            else if (ps2.ps2RX[4] == 83) {
                ps2.digit = 3;
            }
            for (int k = 1; k < 5; k++) {
                for (int l = 0; l < 10; l++) {
                    if (ps2.ps2RX[k] == num[l]) {
                        ps2.gain[k - 1] = l;
                    }
                }
            }
            if (ps2.digit == 1) {
                ps2.ps2YPos = ps2.gain[0];
            }
            else if (ps2.digit == 2) {
                ps2.ps2YPos = (ps2.gain[0] * 10) + ps2.gain[1];
            }
            else if (ps2.digit == 3) {
                ps2.ps2YPos = (ps2.gain[0] * 100) + (ps2.gain[1] * 10) + ps2.gain[2];
            }
        }

        // Convert from 0 - 255 to -128 - 128
//        ps2.pwmOut = ((ps2.ps2Y / 132.0) * 300) + 150;
        ps2.pwmOut = 320 - ps2.ps2YPos;
        if (ps2.pwmOut > 300) {
            ps2.pwmOut = 350;
        }
        if (ps2.pwmOut < -300) {
            ps2.pwmOut = 0;
        }
    }
    // Mode Button
    else if (ps2.mode == 2)
    {
        ps2.on = 1;
        ps2.ps2Y = 0;

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
            ps2.PIDPos += 0.1;
            debounce_time = current_time + 250; // Debounce delay of 250ms
        }
        else if (ps2.l[3] == 1)
        {
            // Decrease ps2.PIDPos
            ps2.PIDPos -= 0.1;
            debounce_time = current_time + 250; // Debounce delay of 250ms
        }
    }
}
