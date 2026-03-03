#ifndef motor_H
#define motor_H
#include "stdint.h"
#include "stm32f1xx_hal.h"

void Motor_Init();
void Motor_Left(int16_t ccrL);
void Motor_Right(int16_t ccrR);
void Motor_Stop();

void reset_motor_controller();
void update_motor_controllers(float steering_adjustment);
float angle_controller(float error_wall_sensor);
float position_controller();

#endif