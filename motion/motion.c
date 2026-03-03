#include "motion.h"
#include "motor.h"
#include "profile.h"
#include "sensor.h"
#include "gyro_6500.h"
#include "encoder.h"
#include <stdint.h>
#include <stm32f1xx_hal.h>

void spin_180(void)
{
	
	Motor_Left(500);
	Motor_Right(-1850);
}

void move_forward(float distance, float top_speed, float end_speed)
{
    Reset_Forward(&forward,0);
	  Reset_Rotation(&rotation,0);                                   // Giu thang
    Start(&forward, distance, top_speed, end_speed, 800);

    while(!is_finished(&forward));                       // CHO DEN KHI XONG

    if(end_speed == 0)
    {
        Motor_Stop();
    }
		reset_motor_controller();
}

void turn_R_smooth(void)
{
    Reset_Forward(&forward,380);
	  Reset_Rotation(&rotation,0); 
    Start(&rotation, -90, 500, 0, 700);
	
    while(!is_finished(&rotation));                      // CHO DEN KHI XONG

    Motor_Stop();
    reset_motor_controller();
}

void turn_L_smooth(void)
{ 
		Reset_Forward(&forward,380);
	  Reset_Rotation(&rotation,0); 
    Start(&rotation, 90, 500, 0, 700);
		
    while(!is_finished(&rotation));                      // CHO DEN KHI XONG

    Motor_Stop();
    reset_motor_controller();
}

