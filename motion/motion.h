#ifndef motion_H
#define motion_H

#include <stdint.h>
#include <stm32f1xx_hal.h>

void turn_L_smooth(void);
void turn_R_smooth(void);
void spin_180(void);
void move_forward(float distance, float top_speed, float end_speed);







#endif