#include "headfile.h"
#include "PID_SpeedControl.h"
#include "MotorVolt.h"

#define SpeedConut_Feq 100
#define PID_SpeedControl_Calc_Feq 1000

#define MAX_Motor_Volt (PWM_DUTY_MAX / 2)
#define MIN_Motor_Volt (-PWM_DUTY_MAX / 2)

bool PID_SpeedControl_On = false;

double P_Value = 2500;//比例项
double D_Value = 6;//微分项


volatile long long encoder1;
volatile long long encoder2;

volatile long long encoder1_last = 0;
volatile long long encoder2_last = 0;

volatile long long delta_encoder1 = 0;
volatile long long delta_encoder2 = 0;

double exp_Speed1 = 0;//电机1的期望速度
double exp_Speed2 = 0;//电机2的期望速度

double Volt1 = 0;
double Volt2 = 0;

volatile double last_E1 = 0;
volatile double last_E2 = 0;
volatile double E1 = 0;
volatile double E2 = 0;

//编码器测速函数，顺便再计算偏差的导数
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
初始化PID速度控制
初始化Encoder1_LSB、Encoder2_LSB的gpio
使用C0、A1外部中断读取编码器
使用TIM_1配置中断测速
使用TIM_3用于电机电压PID控制
*/
void PID_SpeedControl_init()
{
	tim_interrupt_init(TIM_1, SpeedConut_Feq, 1);//初始化定时器中断
	tim_interrupt_init(TIM_3, PID_SpeedControl_Calc_Feq, 2); //初始化基于TIM_3的中断（用于PID）
	gpio_init(Encoder1_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//编码器正反转
	gpio_init(Encoder2_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//编码器正反转
	exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//修改接口后要在isr.h中进行相应修改
	exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//修改接口后要在isr.h中进行相应修改
	PID_SpeedControl_On = true;
}

//电压计算，需要不断循环执行
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
