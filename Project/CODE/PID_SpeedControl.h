#ifndef _PID_SpeedControl_h_
#define _PID_SpeedControl_h_

#define Encoder1_LSB A8
#define Encoder2_LSB C1

//编码器测速函数，顺便再计算偏差的导数
void tim_interrupt_SpeedCount();

/* 
初始化PID速度控制
初始化Encoder1_LSB、Encoder2_LSB的gpio
使用C0、A1外部中断读取编码器
使用TIM_1配置中断测速
使用TIM_3用于电机电压PID控制
*/
void PID_SpeedControl_init();

//电压计算，需要不断循环执行
void PID_Volt_Calc();

void exti_interrupt_encoder_2();
void exti_interrupt_encoder_1();

//是否启用电机转速PID控制
extern bool PID_SpeedControl_On;

extern long long encoder1;//编码器1的值
extern long long encoder2;//编码器2的值
extern long long delta_encoder1;//编码器1的当前值与 1 / SpeedConut_Feq 时间前的值的差
extern long long delta_encoder2;//编码器2的当前值与 1 / SpeedConut_Feq 时间前的值的差
extern double exp_Speed1;//电机1的期望速度，要改变电机转速只需修改此值即可
extern double exp_Speed2;//电机2的期望速度，要改变电机转速只需修改此值即可
extern int Volt1;//电机1的电压，最大值为PWM_DUTY_MAX(50000)
extern int Volt2;//电机2的电压，最大值为PWM_DUTY_MAX(50000)

//电机PID转速控制比例系数
extern double PID_SC_Kp;

//电机PID转速控制积分系数
extern double PID_SC_Ki;

//电机PID转速控制微分系数（未使用）
extern double PID_SC_Kd;

#endif