#include "headfile.h"
#include "MotorVolt.h"

//初始化电机驱动接口
void MotorVolt_init()
{
	gpio_init(DIR1, GPO, GPIO_LOW, GPO_PUSH_PUL);
	gpio_init(DIR2, GPO, GPIO_LOW, GPO_PUSH_PUL);
	pwm_init(TIM_2, TIM_2_CH4_B11, 15000, 0);
	pwm_init(TIM_2, TIM_2_CH2_A12, 15000, 0);

}

//设置电机1的电压，最大值为PWM_DUTY_MAX(50000)
void Motor1_Volt(int duty)
{
	if (duty > 0)
	{
		if (duty > PWM_DUTY_MAX)
		{
			duty = PWM_DUTY_MAX;
		}

		gpio_set(DIR1, 1);
		pwm_duty_updata(TIM_2, TIM_2_CH4_B11, duty);
	}
	else
	{
		if (duty < -PWM_DUTY_MAX)
		{
			duty = -PWM_DUTY_MAX;
		}

		gpio_set(DIR1, 0);
		pwm_duty_updata(TIM_2, TIM_2_CH4_B11, -duty);
	}
}

//设置电机2的电压，最大值为PWM_DUTY_MAX(50000)
void Motor2_Volt(int duty)
{
	if (duty > 0)
	{
		if (duty > PWM_DUTY_MAX)
		{
			duty = PWM_DUTY_MAX;
		}

		gpio_set(DIR2, 1);
		pwm_duty_updata(TIM_2, TIM_2_CH2_A12, duty);
	}
	else
	{
		if (duty < -PWM_DUTY_MAX)
		{
			duty = -PWM_DUTY_MAX;
		}

		gpio_set(DIR2, 0);
		pwm_duty_updata(TIM_2, TIM_2_CH2_A12, -duty);
	}
}