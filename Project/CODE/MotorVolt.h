#ifndef _MotorVolt_H_
#define _MotorVolt_H_

#define DIR1 C9
#define DIR2 A11
#define PWM1 B11
#define PWM2 A12


void MotorVolt_init();

void Motor1_Volt(int);

void Motor2_Volt(int);

#endif