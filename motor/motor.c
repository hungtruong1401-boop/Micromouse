#include "motor.h"
#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "profile.h"
#include "encoder.h"
#include "sensor.h"
#include "gyro_6500.h"
#define Kp_fwd 20.0f
#define Ki_fwd 0
#define Kd_fwd 0.00001f
#define Kp_rot 150.0f
#define Ki_rot 0
#define Kd_rot 0.00225f
#define MOUSE_RADIUS 40 //mm
#define PI 3.14
#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define PWM_TO_V 2.7f

extern TIM_HandleTypeDef htim3;
float g_left_motor_volts;
float g_right_motor_volts;

static float s_old_fwd_error;
static float s_old_rot_error;
static float s_fwd_error;
float s_rot_error;

float v_fwd;
float v_rot;
float left_output = 0;
float right_output = 0;
	
void Motor_Init()
{
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

void Motor_Left(int16_t ccrL)
{
	  
    if(ccrL>0)
    {
        htim3.Instance->CCR1 = 0;
        htim3.Instance->CCR2 = ccrL;
    }
    else if(ccrL<0)
    {
        htim3.Instance->CCR1 = -ccrL;
        htim3.Instance->CCR2 = 0;
    }
    else
    {
        htim3.Instance->CCR1 = 1;
        htim3.Instance->CCR2 = 1;
    }
}

void Motor_Right(int16_t ccrR)
{
		
    if(ccrR>0)
    {
        htim3.Instance->CCR3 = 0;
        htim3.Instance->CCR4 = ccrR;
    }
    else if(ccrR<0)
    {
        htim3.Instance->CCR3 = -ccrR;
        htim3.Instance->CCR4 = 0;
    }
    else
    {
        htim3.Instance->CCR3 = 1;
        htim3.Instance->CCR4 = 1;
    }
}

void Motor_Stop()
{
	Motor_Left(0);
	Motor_Right(0);
}
void reset_motor_controller()
{
  s_fwd_error = 0;
  s_rot_error = 0;
  s_old_fwd_error = 0;
  s_old_rot_error = 0;
}

float position_controller() {
  s_fwd_error += increment(&forward) - Real_Pos_Increment();
  float diff = s_fwd_error - s_old_fwd_error;
  s_old_fwd_error = s_fwd_error;
  float output = Kp_fwd * s_fwd_error + Kd_fwd * diff;
  return output;
}

float angle_controller(float error_wall_sensor) {
  s_rot_error += increment(&rotation) - Real_Ang_Increment();
  s_rot_error += error_wall_sensor;
  error_wall_sensor = s_rot_error;
  float diff = s_rot_error - s_old_rot_error;
  float output = Kp_rot * s_rot_error + Kd_rot * diff;
  return output;
}

void update_motor_controllers(float steering_adjustment)
{
	
  float pos_output = position_controller();
  float rot_output = angle_controller(steering_adjustment);
  left_output = 0;
  right_output = 0;
  left_output += pos_output;
  right_output += pos_output;
  left_output -= rot_output;
  right_output += rot_output;
   v_fwd = forward.speed ;
   v_rot = rotation.speed ;
  float v_left = v_fwd -  v_rot * MOUSE_RADIUS *(PI/180);
  float v_right = v_fwd +  v_rot * MOUSE_RADIUS *(PI/180);
  left_output += v_left * PWM_TO_V;
  right_output += v_right * PWM_TO_V;
	
	left_output = CLAMP(left_output,-1780,1780);
	right_output = 	CLAMP(right_output,-1780,1780);
	
  Motor_Right(right_output);
  Motor_Left(left_output);
}
