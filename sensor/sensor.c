#include "sensor.h"
#include "stm32f1xx_hal.h"
#include "stdint.h"
#include "main.h"
#include "stdbool.h"


#define FR_Sensor 1 //CH2  
#define SR_Sensor 2 //CH3 
#define SL_Sensor 3 //CH4
#define FL_Sensor 4 //CH5
#define STEERING_ADJUST_LIMIT 20
#define LEFT_THRESHOLD  800 //Nguong nhan biet tuong
#define FRONT_THRESHOLD 800 //Nguong nhan biet tuong
#define RIGHT_THRESHOLD 800 //Nguong nhan biet tuong
#define RIGHT_OK        1400//RIGHT MAX (nguong mong muon de di giua tuong)
#define LEFT_OK         1400//LEFT MAX 

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;


static volatile uint16_t sensor_on[NUM_SENSOR];
static volatile uint16_t sensor_off[NUM_SENSOR];
static volatile uint16_t sensor_diff[NUM_SENSOR];
volatile float wall_error = 0; //sai so cam bien tuong dung cho pid

volatile bool g_left_wall_present; //nhan tuong trai
volatile bool g_front_wall_present;//nhan tuong truoc
volatile bool g_right_wall_present;//nhan tuong phai


volatile uint16_t FR;
volatile uint16_t SR;
volatile uint16_t SL;
volatile uint16_t FL;

static void emitter_on(uint8_t emitter)
{
    switch (emitter)
		{   
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
            break;
				case 2:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            break;
				case 3:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
            break;
				case 4:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

static void emitter_off(uint8_t emitter)
{
    switch (emitter) 
		{
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
            break;
				case 2:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            break;
				case 3:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
            break;
				case 4:
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

 static void sm_emitter_adc(void)
{
    static uint8_t emitter_status = 1;
    static uint8_t sensor_index   = 0;

    switch (emitter_status)
    {
        case 1:
            HAL_ADCEx_InjectedStart(&hadc1);
            emitter_status = 2;
            break;

        case 2:            
            sensor_off[sensor_index] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1 + sensor_index);
            emitter_on(sensor_index+1);
            emitter_status = 3;
            break;

        case 3:
            HAL_ADCEx_InjectedStart(&hadc1);
            emitter_status = 4;
            break;

        case 4:
            sensor_on[sensor_index] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1 + sensor_index);
            emitter_off(sensor_index+1); 
            emitter_status = 1;
            if (sensor_index == (NUM_SENSOR - 1))
								sensor_index = 0;
						else
								sensor_index++;
						break;
				default:
						break;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
			   
				sm_emitter_adc();
    }
}

void Get_Sensor_Value(void)
{
	uint8_t i = 0;
  for(i = 0 ;i<NUM_SENSOR ;i++)
	{
		if(sensor_off[i] > sensor_on[i])
		{
			sensor_diff[i]=0;
		}
		else
		{
			sensor_diff[i] = sensor_on[i] - sensor_off[i];
		}
	}
	
	FR = sensor_diff[0];
	SR = sensor_diff[1];
	SL = sensor_diff[2];
	FL = sensor_diff[3];
}

float update_wall_sensor()
{
	g_left_wall_present = SL > LEFT_THRESHOLD;
  g_right_wall_present = SR > RIGHT_THRESHOLD;
  g_front_wall_present = ((FR + FL) * 0.5f) > FRONT_THRESHOLD;
	
	
  float right_error = RIGHT_OK - SR;
  float left_error = LEFT_OK - SL;
  if (g_left_wall_present && g_right_wall_present)
	{
    wall_error = left_error - right_error;
  } 
	else if (g_left_wall_present) 
	{
    wall_error = 2.0 * left_error;
  } 
	else if (g_right_wall_present) 
	{
    wall_error = -2.0 * right_error;
  }
  // the side sensors are not reliable close to a wall ahead.
  // TODO: The magic number 100 may need adjusting
  if (FR > FRONT_THRESHOLD && FL > FRONT_THRESHOLD ) 
	{
    wall_error = 0;
  }
	return wall_error;
}