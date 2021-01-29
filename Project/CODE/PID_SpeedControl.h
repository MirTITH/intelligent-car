#ifndef _PID_SpeedControl_h_
#define _PID_SpeedControl_h_

#define Encoder1_LSB A8
#define Encoder2_LSB C1

void tim_interrupt_SpeedCount();
void PID_SpeedControl_init();

void Motor1_Speed();
void Motor2_Speed();

void PID_Volt_Calc();

void exti_interrupt_encoder_2();
void exti_interrupt_encoder_1();

extern bool PIDOn;
extern volatile long long encoder1;
extern volatile long long encoder2;
extern volatile long long delta_encoder1;
extern volatile long long delta_encoder2;
extern double exp_Speed1;
extern double exp_Speed2;
extern double Volt1;
extern double Volt2;
extern double P_Value;
extern double D_Value;

#endif