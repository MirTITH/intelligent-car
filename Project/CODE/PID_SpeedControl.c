#include "headfile.h"
#include "PID_SpeedControl.h"
#include "MotorVolt.h"

#define SpeedConut_Feq 100
#define PID_SpeedControl_Calc_Feq 1000

#define MAX_Motor_Volt (PWM_DUTY_MAX / 2)
#define MIN_Motor_Volt (-PWM_DUTY_MAX / 2)

bool PID_SpeedControl_On = false;

double P_Value = 2500;//������
double D_Value = 6;//΢����


volatile long long encoder1;
volatile long long encoder2;

volatile long long encoder1_last = 0;
volatile long long encoder2_last = 0;

volatile long long delta_encoder1 = 0;
volatile long long delta_encoder2 = 0;

double exp_Speed1 = 0;//���1�������ٶ�
double exp_Speed2 = 0;//���2�������ٶ�

double Volt1 = 0;
double Volt2 = 0;

volatile double last_E1 = 0;
volatile double last_E2 = 0;
volatile double E1 = 0;
volatile double E2 = 0;

//���������ٺ�����˳���ټ���ƫ��ĵ���
void tim_interrupt_SpeedCount()
{
	delta_encoder1 = encoder1 - encoder1_last;
	delta_encoder2 = encoder2 - encoder2_last;
	encoder1_last = encoder1;
	encoder2_last = encoder2;

	last_E1 = E1;
	last_E2 = E2;
	E1 = exp_Speed1 - delta_encoder1;
	E2 = exp_Speed2 - delta_encoder2;
}

/* 
��ʼ��PID�ٶȿ���
��ʼ��Encoder1_LSB��Encoder2_LSB��gpio
ʹ��C0��A1�ⲿ�ж϶�ȡ������
ʹ��TIM_1�����жϲ���
ʹ��TIM_3���ڵ����ѹPID����
*/
void PID_SpeedControl_init()
{
	tim_interrupt_init(TIM_1, SpeedConut_Feq, 1);//��ʼ����ʱ���ж�
	tim_interrupt_init(TIM_3, PID_SpeedControl_Calc_Feq, 2); //��ʼ������TIM_3���жϣ�����PID��
	gpio_init(Encoder1_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//����������ת
	gpio_init(Encoder2_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//����������ת
	exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//�޸Ľӿں�Ҫ��isr.h�н�����Ӧ�޸�
	exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//�޸Ľӿں�Ҫ��isr.h�н�����Ӧ�޸�
	PID_SpeedControl_On = true;
}

//��ѹ���㣬��Ҫ����ѭ��ִ��
void PID_Volt_Calc()
{
	if (PID_SpeedControl_On)
	{
		Volt1 += P_Value * E1 / PID_SpeedControl_Calc_Feq + D_Value * (E1 - last_E1);
		Volt2 += P_Value * E2 / PID_SpeedControl_Calc_Feq + D_Value * (E2 - last_E2);

		if (Volt1 > MAX_Motor_Volt) Volt1 = MAX_Motor_Volt;
		if (Volt1 < MIN_Motor_Volt) Volt1 = MIN_Motor_Volt;
		if (Volt2 > MAX_Motor_Volt) Volt2 = MAX_Motor_Volt;
		if (Volt2 < MIN_Motor_Volt) Volt2 = MIN_Motor_Volt;

		Motor1_Volt(Volt1);
		Motor2_Volt(Volt2);
	}
}

void exti_interrupt_encoder_2()
{
	if (gpio_get(Encoder2_LSB))
	{
		encoder2--;
	}
	else
	{
		encoder2++;
	}
	
}

void exti_interrupt_encoder_1()
{
	if (gpio_get(Encoder1_LSB))
	{
		encoder1++;
	}
	else
	{
		encoder1--;
	}
	
}
