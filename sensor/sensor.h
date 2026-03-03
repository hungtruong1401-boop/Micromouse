#ifndef sensor_H
#define sensor_H
#include "stm32f1xx_hal.h"
#include "stdint.h"
#define NUM_SENSOR 4

void Get_Sensor_Value(void);
float update_wall_sensor(void);

#endif