/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				main
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
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


// *************************** ����˵�� ***************************
// 
// ������Ҫ׼����ɿƼ�MM32SPIIN27PS���İ�һ��
// 
// ����������Ҫ׼����ɿƼ�CMSIS-DAP��Jlink����������һ��
// 
// �������Ǹ��չ��� ������ͬѧ����ֲʹ��
// 
// ���µĹ��̻��߹����ƶ���λ�����ִ�����²���
// ��һ�� �ر��������д򿪵��ļ�
// �ڶ��� project  clean  �ȴ��·�����������
// 
// *************************** ����˵�� ***************************

// **************************** �궨�� ****************************

const double PI = 3.1415926535897932384626433832795;
#define Encoder1_LSB A8
#define Encoder2_LSB C1

// **************************** �궨�� ****************************

// **************************** �������� ****************************
uint16 loop_count = 0;
uint8 io_input_state = 0;
long long encoder2 = 0;
long long encoder1 = 0;

uint64_t dt = 0;//����ѭ��ʱ����(us)
uint64_t lastT = 0;//��һ��ѭ��ʱ��(us)

double acc_angle_yz = 0;//y,z�᷽����ٶȵĽǶ�

const double bal_acc_angle_yz = 0.81;//ƽ��ʱ�ĽǶ�

double angle_yz_err = 0;

int Volt_all = 0;
// **************************** �������� ****************************

// **************************** �������� ****************************
int main(void)
{
	board_init(true);																// ��ʼ�� debug �������
	icm20602_init_spi();//��ʼ������ģ��

	//gpio_init(A1, GPI, GPIO_LOW, GPI_FLOATING_IN);
	gpio_init(Encoder1_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//����������ת
	//gpio_init(C0, GPI, GPIO_LOW, GPI_FLOATING_IN);
	gpio_init(Encoder2_LSB, GPI, GPIO_LOW, GPI_FLOATING_IN);//����������ת

	exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//�޸Ľӿں�Ҫ��isr.h�н�����Ӧ�޸�
	exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//�޸Ľӿں�Ҫ��isr.h�н�����Ӧ�޸�

	MotorVolt_init();

	PID_SpeedControl_init();

	Motor1_Volt(0);
	Motor2_Volt(0);

	

	while(1)
	{
		//�˴���д��Ҫѭ��ִ�еĴ���
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
		//�˴���д��Ҫѭ��ִ�еĴ���
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
// **************************** �������� ****************************
