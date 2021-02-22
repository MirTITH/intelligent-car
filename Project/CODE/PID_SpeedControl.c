#include "headfile.h"
#include "PID_SpeedControl.h"

#define SpeedConut_Period 1000

extern long long encoder1;
extern long long encoder2;

long long encoder1_last = 0;
long long encoder2_last = 0;

long long delta_encoder1 = 0;
long long delta_encoder2 = 0;

//���������ٺ���
void tim_interrupt_SpeedCount()
{
	delta_encoder1 = encoder1 - encoder1_last;
	delta_encoder2 = encoder2 - encoder2_last;
	encoder1_last = encoder1;
	encoder2_last = encoder2;
}

/* 
��ʼ��PID�ٶȿ���
ʹ��TIM_1�����жϲ���
*/
void PID_SpeedControl_init()
{
	tim_interrupt_init(TIM_1, SpeedConut_Period, 1);//��ʼ����ʱ���ж�
}

void 