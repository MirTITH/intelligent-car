/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				main
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32SPIN2XPs
* @Taobao			https://seekfree.taobao.com/
* @date				2020-11-23
********************************************************************************************************************/

#include "headfile.h"
#include "MotorVolt.h"
#include "PID_SpeedControl.h"
extern long long delta_encoder1;
extern long long delta_encoder2;


// *************************** 例程说明 ***************************
// 
// 测试需要准备逐飞科技MM32SPIIN27PS核心板一块
// 
// 调试下载需要准备逐飞科技CMSIS-DAP或Jlink调试下载器一个
// 
// 本例程是个空工程 用来给同学们移植使用
// 
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project  clean  等待下方进度条走完
// 
// *************************** 例程说明 ***************************

// **************************** 宏定义 ****************************

const double PI = 3.1415926535897932384626433832795;
#define Encoder1_LSB A8
#define Encoder2_LSB C1

// **************************** 宏定义 ****************************

// **************************** 变量定义 ****************************
uint16 loop_count = 0;
uint8 io_input_state = 0;
long long encoder2 = 0;
long long encoder1 = 0;

uint64_t dt = 0;//两次循环时间间隔(us)
uint64_t lastT = 0;//上一次循环时间(us)

double acc_angle_yz = 0;//y,z轴方向加速度的角度

const double bal_acc_angle_yz = 0.81;//平衡时的角度

double angle_yz_err = 0;

int Volt_all = 0;
// **************************** 变量定义 ****************************

// **************************** 代码区域 ****************************
int main(void)
{
	board_init(true);																// 初始化 debug 输出串口
	icm20602_init_spi();//初始化无线模块

	//gpio_init(A1, GPI, GPIO_LOW, GPI_FLOATING_IN);
	gpio_init(Encoder1_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//编码器正反转
	//gpio_init(C0, GPI, GPIO_LOW, GPI_FLOATING_IN);
	gpio_init(Encoder2_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//编码器正反转

	exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//修改接口后要在isr.h中进行相应修改
	exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//修改接口后要在isr.h中进行相应修改

	MotorVolt_init();

	PID_SpeedControl_init();

	Motor1_Volt(0);
	Motor2_Volt(0);

	

	while(1)
	{
		//此处编写需要循环执行的代码
		//dt = systick_getval_us() - lastT;
		//lastT = systick_getval_us();
		//PrintData();
		//systick_start();
		systick_delay_ms(1000);
		
		get_icm20602_gyro_spi();
		get_icm20602_accdata_spi();

		if (icm_acc_z != 0)
		{
			acc_angle_yz = atan((double)icm_acc_y / icm_acc_z);
		}
		else
		{
			acc_angle_yz = PI / 2;
		}
		angle_yz_err = bal_acc_angle_yz - acc_angle_yz;
		
		

/* 		Volt_all = angle_yz_err * 12000;

		Motor1_Volt(Volt_all);
		Motor2_Volt(Volt_all);

		if (Volt_all > PWM_DUTY_MAX / 2|| Volt_all < -PWM_DUTY_MAX / 2)
		{
			Motor1_Volt(0);
			Motor2_Volt(0);
			while (1);
		} */
		//此处编写需要循环执行的代码
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

void PrintData1()
{
	//printf("Hello\n");
	//printf("A1 %d,A8 %d,C0 %d,C1 %d\n",gpio_get(A1), gpio_get(A8), gpio_get(C0), gpio_get(C1));
	//printf("M2 %lld,M1 %lld\n", encoder2, encoder1);
	//printf("rx %d,ry %d,rz %d,ax %d,ay %d,az %d\n",icm_gyro_x, icm_gyro_y, icm_gyro_z, icm_acc_x, icm_acc_y, icm_acc_z);
	//printf("%lf\n", angle_yz_err);
	//printf("S1 %lld,S2 %lld,dt %llu\n", delta_encoder1, delta_encoder2, systick_getval_us());
	printf("%lld\n", systick_getval_ms());
}
// **************************** 代码区域 ****************************
