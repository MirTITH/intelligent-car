#include "headfile.h"
#include "PID_SpeedControl.h"
#include "MotorVolt.h"

#define SpeedConut_Feq 100
#define PID_SpeedControl_Calc_Feq 1000

#define MAX_Motor_Volt (PWM_DUTY_MAX)
#define MIN_Motor_Volt (-PWM_DUTY_MAX)

bool PID_SpeedControl_On = false;

double PID_SC_Kp = 400;
double PID_SC_Ki = 2000;
double PID_SC_Kd = 0;


long long encoder1;
long long encoder2;

long long encoder1_last = 0;
long long encoder2_last = 0;

long long delta_encoder1 = 0;
long long delta_encoder2 = 0;

double exp_Speed1 = 0;
double exp_Speed2 = 0;

int Volt1 = 0;
int Volt2 = 0;

double last_E1 = 0;
double last_E2 = 0;
double E1 = 0;
double E2 = 0;

void tim_interrupt_SpeedCount()
{
	delta_encoder1 = (encoder1 - encoder1_last) * SpeedConut_Feq / 100;
	delta_encoder2 = (encoder2 - encoder2_last) * SpeedConut_Feq / 100;
	encoder1_last = encoder1;
	encoder2_last = encoder2;

	last_E1 = E1;
	last_E2 = E2;
	E1 = exp_Speed1 - delta_encoder1;
	E2 = exp_Speed2 - delta_encoder2;
}

void PID_SpeedControl_init()
{
	tim_interrupt_init(TIM_1, SpeedConut_Feq, 0);//初始化定时器中断
	tim_interrupt_init(TIM_3, PID_SpeedControl_Calc_Feq, 2); //初始化基于TIM_3的中断（用于PID）
	gpio_init(Encoder1_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//编码器正反转
	gpio_init(Encoder2_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//编码器正反转
	exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//修改接口后要在isr.h中进行相应修改
	exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//修改接口后要在isr.h中进行相应修改
	PID_SpeedControl_On = true;
}

void PID_Volt_Calc()
{
	if (PID_SpeedControl_On)
	{
		static double I_Value1 = 0;//电机1积分项的值
		static double I_Value2 = 0;//电机2积分项的值

		if (PID_SC_Ki == 0)
		{
			I_Value1 = 0;
			I_Value2 = 0;
		}
		else
		{
			I_Value1 += PID_SC_Ki * E1 / PID_SpeedControl_Calc_Feq;
			I_Value2 += PID_SC_Ki * E2 / PID_SpeedControl_Calc_Feq;
		}
		
		Volt1 = PID_SC_Kp * E1 + I_Value1;
		Volt2 = PID_SC_Kp * E2 + I_Value2;

		if (Volt1 > MAX_Motor_Volt) Volt1 = MAX_Motor_Volt;
		if (Volt1 < MIN_Motor_Volt) Volt1 = MIN_Motor_Volt;
		if (Volt2 > MAX_Motor_Volt) Volt2 = MAX_Motor_Volt;
		if (Volt2 < MIN_Motor_Volt) Volt2 = MIN_Motor_Volt;

		Motor1_Volt(Volt1);
		Motor2_Volt(Volt2);
	}
}

