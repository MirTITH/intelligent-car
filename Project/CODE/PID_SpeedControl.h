#ifndef _PID_SpeedControl_h_
#define _PID_SpeedControl_h_

#define Encoder1_LSB A8
#define Encoder2_LSB C1

//���������ٺ�����˳���ټ���ƫ��ĵ���
void tim_interrupt_SpeedCount();

/* 
��ʼ��PID�ٶȿ���
��ʼ��Encoder1_LSB��Encoder2_LSB��gpio
ʹ��C0��A1�ⲿ�ж϶�ȡ������
ʹ��TIM_1�����жϲ���
ʹ��TIM_3���ڵ����ѹPID����
*/
void PID_SpeedControl_init();

//��ѹ���㣬��Ҫ����ѭ��ִ��
void PID_Volt_Calc();

void exti_interrupt_encoder_2();
void exti_interrupt_encoder_1();

//�Ƿ����õ��ת��PID����
extern bool PID_SpeedControl_On;

extern long long encoder1;//������1��ֵ
extern long long encoder2;//������2��ֵ
extern long long delta_encoder1;//������1�ĵ�ǰֵ�� 1 / SpeedConut_Feq ʱ��ǰ��ֵ�Ĳ�
extern long long delta_encoder2;//������2�ĵ�ǰֵ�� 1 / SpeedConut_Feq ʱ��ǰ��ֵ�Ĳ�
extern double exp_Speed1;//���1�������ٶȣ�Ҫ�ı���ת��ֻ���޸Ĵ�ֵ����
extern double exp_Speed2;//���2�������ٶȣ�Ҫ�ı���ת��ֻ���޸Ĵ�ֵ����
extern int Volt1;//���1�ĵ�ѹ�����ֵΪPWM_DUTY_MAX(50000)
extern int Volt2;//���2�ĵ�ѹ�����ֵΪPWM_DUTY_MAX(50000)

//���PIDת�ٿ��Ʊ���ϵ��
extern double PID_SC_Kp;

//���PIDת�ٿ��ƻ���ϵ��
extern double PID_SC_Ki;

//���PIDת�ٿ���΢��ϵ����δʹ�ã�
extern double PID_SC_Kd;

#endif