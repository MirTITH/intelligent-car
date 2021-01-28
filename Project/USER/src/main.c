#include "headfile.h"
#include "MotorVolt.h"//定义了驱动板引脚
#include "PID_SpeedControl.h"//定义了编码器引脚
#include <stdlib.h>

// **************************** 宏定义 ****************************

const double PI = 3.1415926535897932384626433832795;

// **************************** 宏定义 ****************************

// **************************** 变量定义 ****************************

uint32_t dt = 0;//两次循环的间隔时间（微秒）
uint32_t total_us = 0;//总时间

double acc_angle_yz = 0;//y,z轴方向加速度的角度
const double bal_acc_angle_yz = 0.81;//平衡时的角度
double angle_yz_err = 0;


// **************************** 变量定义 ****************************

// **************************** 代码区域 ****************************

void PrintData();
void Update_Gyro_Acc();
char* getrxs();
void Clear_rx_buffer();
void GetSpeedFromRX();
int IsCharNum(char*);

int main(void)
{
	board_init(true);																// 初始化 debug 输出串口
	seekfree_wireless_init();
	icm20602_init_spi();
	MotorVolt_init();
	PID_SpeedControl_init();

	exp_Speed1 = 500;
	exp_Speed2 = 500;

	while(1)
	{
		PrintData();
		dt = systick_getval_us();
		total_us += dt;
		systick_start();
		//systick_delay_ms(1);
		Update_Gyro_Acc();

/* 		if (total_us % 4000000 > 2000000)
		{
			exp_Speed1 = 50;
			exp_Speed2 = 50;
		}
		else
		{
			exp_Speed1 = -50;
			exp_Speed2 = -50;
		} */

		GetSpeedFromRX();
		
	}
}


void PrintData()
{
	static unsigned int counter = 0;
	if (counter--) return;
	counter = 100;
	//printf("Hello\n");
	//printf("A1 %d,A8 %d,C0 %d,C1 %d\n",gpio_get(A1), gpio_get(A8), gpio_get(C0), gpio_get(C1));
	//printf("M2 %lld,M1 %lld", encoder2, encoder1);
	//printf("rx %d,ry %d,rz %d,ax %d,ay %d,az %d\n",icm_gyro_x, icm_gyro_y, icm_gyro_z, icm_acc_x, icm_acc_y, icm_acc_z);
	//printf("%lf\n", angle_yz_err);
	printf(",eS1 %lf,eS2 %lf", exp_Speed1, exp_Speed2);
	printf(",S1 %lld,S2 %lld", delta_encoder1, delta_encoder2);
	//printf(",v1 %lf,v2 %lf", Volt1 / 100, Volt2 / 100);
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

//更新加速度、角度值
void Update_Gyro_Acc()
{
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
}

//获取串口接收到的字符串（以'\0'或回车结尾）
char* getrxs()
{
	if (wireless_rx_buffer[wireless_rx_index - 1] != '\r' && wireless_rx_buffer[wireless_rx_index - 1] != '\n' && wireless_rx_buffer[wireless_rx_index - 1] != '\0')
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

	while (wireless_rx_buffer[buffer_cursor] != '\r' && wireless_rx_buffer[buffer_cursor] != '\0')
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
void GetSpeedFromRX()
{
	char* str = getrxs();
	if (str == NULL)
	{
		return;
	}

	static const char* Speed1_str = "s1=";
	static const char* Speed2_str = "s2=";
	static const char* Speed_str = "s=";

	int num_end = 0;
	char* str_pointer = 0;
	char str_num[WIRELESS_BUFFER_SIZE] = {0};

	//获取exp_Speed1
	str_pointer = strstr(str, Speed1_str);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(Speed1_str);
		num_end = 0;
		while (IsCharNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';

		exp_Speed1 = atoi(str_num);
	}
	

	//获取exp_Speed2
	str_pointer = strstr(str, Speed2_str);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(Speed2_str);
		num_end = 0;
		while (IsCharNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';

		exp_Speed2 = atoi(str_num);
	}

	//获取总速度
	str_pointer = strstr(str, Speed_str);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(Speed_str);
		num_end = 0;
		while (IsCharNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';

		exp_Speed1 = atoi(str_num);
		exp_Speed2 = atoi(str_num);
	}
}

//判断字符是否属于数字或正负号
int IsCharNum(char* cha)
{
	return ((*cha >= '0' && *cha <= '9') || *cha == '-' || *cha == '+');
}

// **************************** 代码区域 ****************************
