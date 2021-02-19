#include "headfile.h"
#include "MotorVolt.h"//定义了驱动板引脚
#include "PID_SpeedControl.h"//定义了编码器引脚
#include "PID_AngleControl.h"
#include "Timer_us.h"
#include <stdlib.h>

// **************************** 宏定义 ****************************

// **************************** 宏定义 ****************************

// **************************** 变量定义 ****************************


// **************************** 变量定义 ****************************

// **************************** 代码区域 ****************************

void PrintData();

char* getrxs();
void Clear_rx_buffer();
void GetInfoFromRX();
int IsChariNum(char*);
int IsCharfNum(char*);
void GetiValueFromStr(int*, char*, char*);
void GetfValueFromStr(double*, char*, char*);
unsigned long long lastT = 0;//上次循环的微秒值
unsigned long long dt = 0;//循环的间隔（微秒）



int main(void)
{
	board_init(true);																// 初始化 debug 输出串口
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
		//systick_delay_ms(10);
		
/* 		if (Timer_us_Get() % 5000000 > 2500000)
		{
			exp_Speed1 = 50;
			exp_Speed2 = 50;
		}
		else
		{
			exp_Speed1 = -50;
			exp_Speed2 = -50;
		}
 */
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
		// printf(",%lf", speed_car);
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

//获取串口接收到的字符串（以'\0'或'\r'或'\n'结尾）
char* getrxs()
{
	if (!(wireless_rx_buffer[wireless_rx_index - 1] == '\r' || wireless_rx_buffer[wireless_rx_index - 1] == '\n' || wireless_rx_buffer[wireless_rx_index - 1] == '\0'))
	{
		return NULL;
	}
	int buffer_cursor = wireless_rx_index;
	bool circulate_flag = false;
	static char result[WIRELESS_BUFFER_SIZE + 1] = {0};
	buffer_cursor -= 2;
	if (buffer_cursor < 0)
	{
		buffer_cursor += WIRELESS_BUFFER_SIZE;
	}

	while (!(wireless_rx_buffer[buffer_cursor] == '\r' || wireless_rx_buffer[buffer_cursor] == '\0' || wireless_rx_buffer[buffer_cursor] == '\n'))
	{
		buffer_cursor--;

		if (buffer_cursor < 0)
		{
			if (circulate_flag == true)
			{
				return NULL;
			}
			circulate_flag = true;
			buffer_cursor += WIRELESS_BUFFER_SIZE;
		}
	}

	for (int i = 0; i <= WIRELESS_BUFFER_SIZE; i++)
	{
		buffer_cursor++;
		if (buffer_cursor >= WIRELESS_BUFFER_SIZE)
		{
			buffer_cursor -= WIRELESS_BUFFER_SIZE;
		}

		if (buffer_cursor == wireless_rx_index)
		{
			result[i] = '\0';
			break;
		}
		result[i] = wireless_rx_buffer[buffer_cursor];
	}

	Clear_rx_buffer();
	return result;
}

//清除串口接收缓冲区
void Clear_rx_buffer()
{
	if (wireless_rx_index - 2 < 0)
	{
		wireless_rx_buffer[WIRELESS_BUFFER_SIZE + wireless_rx_index - 2] = '\0';
	}
	else
	{
		wireless_rx_buffer[wireless_rx_index - 2] = '\0';
	}
	
}

//从串口获取exp_Speed
void GetInfoFromRX()
{
	char* str = getrxs();
	if (str == NULL)
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

	GetfValueFromStr(&exp_Speed1, str, "es1=");
	GetfValueFromStr(&exp_Speed2, str, "es2=");

	GetfValueFromStr(&exp_Speed1, str, "es=");
	GetfValueFromStr(&exp_Speed2, str, "es=");

	//获取PID_SpeedControl的参数
	GetfValueFromStr(&PID_SC_Kp, str, "sp=");
	GetfValueFromStr(&PID_SC_Ki, str, "si=");
	GetfValueFromStr(&PID_SC_Kd, str, "sd=");

	//获取PID_AngleControl的参数
	GetfValueFromStr(&AngleControl_P, str, "ap=");
	GetfValueFromStr(&AngleControl_I, str, "ai=");
	GetfValueFromStr(&AngleControl_D, str, "ad=");
	GetfValueFromStr(&bal_acc_angle_yz, str, "ab=");

	GetfValueFromStr(&acc_ratio, str, "ar=");
	GetfValueFromStr(&gyro_ratio, str, "gr=");

	GetfValueFromStr(&AC_CarSpeed_P, str, "cp=");
	GetfValueFromStr(&AC_CarSpeed_D, str, "cd=");
	GetfValueFromStr(&speed_car, str, "s=");
	GetfValueFromStr(&turnRatio, str, "r=");




}

//判断字符是否属于整数或正负号
int IsChariNum(char* cha)
{
	return ((*cha >= '0' && *cha <= '9') || *cha == '-' || *cha == '+');
}

//判断字符是否属于浮点数或正负号
int IsCharfNum(char* cha)
{
	return ((*cha >= '0' && *cha <= '9') || *cha == '-' || *cha == '+' || *cha == '.');
}

/*
从str（长度为WIRELESS_BUFFER_SIZE）中寻找subStr后面的整数并将num_set设为该整数
*/
void GetiValueFromStr(int* num_set, char* str, char* subStr)
{
	int num_end = 0;
	char* str_pointer = 0;
	char str_num[WIRELESS_BUFFER_SIZE] = {0};

	str_pointer = strstr(str, subStr);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(subStr);
		num_end = 0;
		while (IsChariNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';
		*num_set = atoi(str_num);
	}
}

/*
从str（长度为WIRELESS_BUFFER_SIZE）中寻找subStr后面的浮点数并将num_set设为该浮点数
*/
void GetfValueFromStr(double* num_set, char* str, char* subStr)
{
	int num_end = 0;
	char* str_pointer = 0;
	char str_num[WIRELESS_BUFFER_SIZE] = {0};

	str_pointer = strstr(str, subStr);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(subStr);
		num_end = 0;
		while (IsCharfNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';

		*num_set = atof(str_num);
	}
}


// **************************** 代码区域 ****************************
