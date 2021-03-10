#ifndef _MotorVolt_H_
#define _MotorVolt_H_

#define DIR1 C9
#define DIR2 A11
#define PWM1 B11
#define PWM2 A12

//初始化电机驱动接口
void MotorVolt_init();

//设置电机1的电压，最大值为PWM_DUTY_MAX(50000)
void Motor1_Volt(int);

//设置电机2的电压，最大值为PWM_DUTY_MAX(50000)
void Motor2_Volt(int);

#endif