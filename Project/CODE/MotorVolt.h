#ifndef _MotorVolt_H_
#define _MotorVolt_H_

#define DIR1 C9
#define DIR2 A11
#define PWM1 B11
#define PWM2 A12

//��ʼ����������ӿ�
void MotorVolt_init();

//���õ��1�ĵ�ѹ�����ֵΪPWM_DUTY_MAX(50000)
void Motor1_Volt(int);

//���õ��2�ĵ�ѹ�����ֵΪPWM_DUTY_MAX(50000)
void Motor2_Volt(int);

#endif