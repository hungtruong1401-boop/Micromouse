#ifndef encoder_H
#define encoder_H
#include <stdint.h>
#define MM_PER_CNT_ENCODER 0.08206f   //1 vòng 820 xung encoder , duong kinh banh 20mm

void Encoder_Init();
void Encoder_Reset();
void Encoder_Update();
int16_t Get_Encoder_Left(void);
int16_t Get_Encoder_Right(void);
	
float Real_Position();
float Real_Pos_Increment();

#endif