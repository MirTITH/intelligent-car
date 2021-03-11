#include "headfile.h"
#include "MySerial.h"
#include "MotorVolt.h"//����������������
#include "PID_SpeedControl.h"//�����˱���������
#include "PID_AngleControl.h"
#include "Timer_us.h"
#include <stdlib.h>

// **************************** �궨�� ****************************

// **************************** �궨�� ****************************

// **************************** �������� ****************************


// **************************** �������� ****************************

// **************************** �������� ****************************

void PrintData();
void GetInfoFromRX();


unsigned long long lastT = 0;//�ϴ�ѭ����΢��ֵ
unsigned long long dt = 0;//ѭ���ļ����΢�룩



int main(void)
{
	board_init(true);																// ��ʼ�� debug �������
	seekfree_wireless_init();
	icm20602_init_spi();
	MotorVolt_init();
	PID_SpeedControl_init();
	PID_AngleControl_init();
	Timer_us_init();

	while(1)
	{
		dt = Timer_us_Get() - lastT;
		lastT = Timer_us_Get();

		PrintData();
		systick_delay_ms(10);
		
		GetInfoFromRX();
	}
}

void PrintData()
{
	static unsigned int counter = 0;
	if (counter--) return;
	counter = 10;
	//printf("Hello\n");
	//printf("A1 %d,A8 %d,C0 %d,C1 %d\n",gpio_get(A1), gpio_get(A8), gpio_get(C0), gpio_get(C1));
	//printf("M2 %lld,M1 %lld", encoder2, encoder1);
	// printf("wx %d",icm_gyro_x);
	// printf(",ax %d,ay %d,az %d", icm_acc_x, icm_acc_y, icm_acc_z);
	// printf(",rx %lld,ry %lld,rz %lld", rx, ry, rz);
	if (PID_SpeedControl_On)
	{
		printf("E1,E2,s1,s2,Dab\n");
		// printf("Hubu,a\n");
		printf("%.2lf", exp_Speed1);
		printf(",%.2lf", exp_Speed2);
		printf(",%lld,%lld", delta_encoder1, delta_encoder2);
		// printf(",%lf", car_speed);
		// printf(",%lf", angle_yz_err * 10);
		printf(",%lf", 10 * bal_acc_angle_yz);
		// printf(",%lf", (bal_acc_angle_yz - acc_angle_yz) * 10);
		//printf(",v1 %d,v2 %d", Volt1 / 100, Volt2 / 100);
		// printf(",p %lf,d %lf", PID_SC_Kp, PID_SC_Kd);
		// printf(",dt %llu", dt);
	}else
	{
		//printf("p,d,ap,ai,ad\n");
		printf("p %lf,i %lf,d %lf", PID_SC_Kp, PID_SC_Ki, PID_SC_Kd);
		printf(",ap %lf,ai %lf,ad %lf", AngleControl_P, AngleControl_I, AngleControl_D);
		printf(",ab %lf", bal_acc_angle_yz);
		printf(",E %lf", angle_yz_err);
		printf(",cp %lf,cd %lf", AC_CarSpeed_P, AC_CarSpeed_D);
		// printf(",ar %lf, gr %e", acc_ratio, gyro_ratio);


	}
	

	// printf("%d", PID_SpeedControl_On);
	//printf(",dt %u\n", dt);
/* 	for (int i = 0; i < WIRELESS_BUFFER_SIZE; i++)
	{
		if (i == wireless_rx_index)
		{
			printf("%x-", wireless_rx_buffer[i]);
		}
		else
		{
			printf("%x ", wireless_rx_buffer[i]);
		}	
	}
	 */
	//printf("%s", getrxs());

	printf("\n");
}

void GetInfoFromRX()
{
	char* str = getrxs();
	if (str == NULL)
	{
		return;
	}

	if (str[0] == '\r' || str[0] == '\n' || str[0] == '\0')
	{
		return;
	}

	if (strchr(str, '\'') != NULL) 
	{
		PID_SpeedControl_On = false;
		PID_AngleControl_On = false;
		Motor1_Volt(0);
		Motor2_Volt(0);
		PID_AC_I_Value = 0;
		return;
	}

	if (strchr(str, '$') != NULL) 
	{
		PID_SpeedControl_On = true;
		PID_AngleControl_On = true;
		return;
	}

	// GetfValueFromStr(&exp_Speed1, str, "es1=");
	// GetfValueFromStr(&exp_Speed2, str, "es2=");

	// GetfValueFromStr(&exp_Speed1, str, "es=");
	// GetfValueFromStr(&exp_Speed2, str, "es=");

	GetfValueFromStr(&car_speed, str, "s=");
	GetfValueFromStr(&turnRatio, str, "r=");

	//��ȡPID_SpeedControl�Ĳ���
	GetfValueFromStr(&PID_SC_Kp, str, "sp=");
	GetfValueFromStr(&PID_SC_Ki, str, "si=");
	GetfValueFromStr(&PID_SC_Kd, str, "sd=");

	//��ȡPID_AngleControl�Ĳ���
	GetfValueFromStr(&AngleControl_P, str, "ap=");
	GetfValueFromStr(&AngleControl_I, str, "ai=");
	GetfValueFromStr(&AngleControl_D, str, "ad=");
	GetfValueFromStr(&bal_acc_angle_yz, str, "ab=");

	GetfValueFromStr(&acc_ratio, str, "ar=");
	GetfValueFromStr(&gyro_ratio, str, "gr=");

	GetfValueFromStr(&AC_CarSpeed_P, str, "cp=");
	GetfValueFromStr(&AC_CarSpeed_D, str, "cd=");
	//printf("%s", str);

}

// **************************** �������� ****************************
