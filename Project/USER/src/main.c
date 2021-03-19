#include "headfile.h"
#include "MySerial.h"
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
void GetInfoFromRX();

typedef struct CAM_RESULT
{
	int board_left;
	int board_right;
	bool found_left;
	bool found_right;
} CamResult;

/*
row 搜索的行，（从上往下递增）
center 从哪里开始往两边搜索
step 搜索的步长
threshold 阈值
*/
CamResult Camera_Calc(int row, int center, int step, int threshold);


unsigned long long lastT = 0;//上次循环的微秒值
unsigned long long dt = 0;//循环的间隔（微秒）

int midline_40;  // 其实是左边界+右边界，是中心值的2倍;
int CamTurnRate = 168;
CamResult cam_result_40;

// int board_right;
// int board_left;

int main(void)
{
	board_init(true);																// 初始化 debug 输出串口
	seekfree_wireless_init();
	ips114_init();
	icm20602_init_spi();
	mt9v03x_init();
	MotorVolt_init();
	PID_SpeedControl_init();
	PID_AngleControl_init();
	
	Timer_us_init();

	gpio_init(A6, GPO, GPIO_LOW, GPO_PUSH_PULL); // 舵机
	pwm_init(TIM_17, TIM_17_CH1_A07, 50, 0); // 舵机

	int screen_counter = 10;
	

	// unsigned long long TBegin;

	while(1)
	{
		// dt = Timer_us_Get() - lastT;
		// lastT = Timer_us_Get();

		PrintData();
		// systick_delay_ms(10);

		if(mt9v03x_finish_flag)
		{
			// TBegin = Timer_us_Get();

			cam_result_40 = Camera_Calc(40, MT9V03X_W / 2, 5, 12);
			midline_40 = cam_result_40.board_left + cam_result_40.board_right;

			if (cam_result_40.found_left && cam_result_40.found_right)
			{
				turnRatio = -(double)(midline_40 - 168) / CamTurnRate;
			}
			else if (cam_result_40.found_left && !cam_result_40.found_right)
			{
				turnRatio = -(double)(midline_40 - 168) / CamTurnRate;
			}
			else if (!cam_result_40.found_left && cam_result_40.found_right)
			{
				turnRatio = -(double)(midline_40 - 168) / CamTurnRate;
			}
			else if (!cam_result_40.found_left && !cam_result_40.found_right)
			{
				turnRatio = 0;
			}

			if (screen_counter > 0)
			{
				ips114_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
				screen_counter--;
			}
			//seekfree_sendimg_03x(UART_1, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
			mt9v03x_finish_flag = 0;

			// printf("%lld\n", Timer_us_Get() - TBegin);
		}

		if (turnRatio > 0.8) turnRatio = 0.8;
		if (turnRatio < -0.8) turnRatio = -0.8;

		GetInfoFromRX();
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
	// printf("wx %d",icm_gyro_x);
	// printf(",ax %d,ay %d,az %d", icm_acc_x, icm_acc_y, icm_acc_z);
	// printf(",rx %lld,ry %lld,rz %lld", rx, ry, rz);
	if (PID_SpeedControl_On)
	{
		// printf("s1,s2\n");
		// printf("Hubu,a\n");
		// printf("%.2lf", exp_Speed1);
		// printf(",%.2lf", exp_Speed2);
		// printf(",%d,%d", delta_encoder1, delta_encoder2);
		printf("m %d,",midline_40);
		printf("R %llf", turnRatio);
		printf(",%d%d", cam_result_40.found_left, cam_result_40.found_right);
		// printf(",%lf", car_speed);
		// printf(",%lf", angle_yz_err * 10);
		// printf(",%lf", 10 * exp_acc_angle_yz);
		// printf(",%lf", (exp_acc_angle_yz - acc_angle_yz) * 10);
		// printf(",%d,%d", Volt1 / 100, Volt2 / 100);
		// printf(",%.2lf", PID_AC_D_Value);
		// printf(",p %lf,d %lf", PID_SC_Kp, PID_SC_Kd);
		// printf(",dt %llu", dt);
	}else
	{
		//printf("p,d,ap,ai,ad\n");
		printf("p %lf,i %lf,d %lf", PID_SC_Kp, PID_SC_Ki, PID_SC_Kd);
		printf(",ap %lf,ai %lf,ad %lf", AngleControl_P, AngleControl_I, AngleControl_D);
		printf(",ab %lf", exp_acc_angle_yz);
		printf(",E %lf", angle_yz_err);
		printf(",cp %lf,cd %lf", AC_CarSpeed_P, AC_CarSpeed_D);
		printg(",ct %d", CamTurnRate);
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
	// GetfValueFromStr(&turnRatio, str, "r=");

	//获取PID_SpeedControl的参数
	GetfValueFromStr(&PID_SC_Kp, str, "sp=");
	GetfValueFromStr(&PID_SC_Ki, str, "si=");
	GetfValueFromStr(&PID_SC_Kd, str, "sd=");

	//获取PID_AngleControl的参数
	GetfValueFromStr(&AngleControl_P, str, "ap=");
	GetfValueFromStr(&AngleControl_I, str, "ai=");
	GetfValueFromStr(&AngleControl_D, str, "ad=");
	GetfValueFromStr(&exp_acc_angle_yz, str, "ab=");

	GetfValueFromStr(&acc_ratio, str, "ar=");
	GetfValueFromStr(&gyro_ratio, str, "gr=");

	GetfValueFromStr(&AC_CarSpeed_P, str, "cp=");
	GetfValueFromStr(&AC_CarSpeed_D, str, "cd=");

	GetiValueFromStr(&CamTurnRate, str, "ct=");

	//printf("%s", str);

}

CamResult Camera_Calc(int row, int center, int step, int threshold)
{
	CamResult result;
	result.board_left = -1;
	result.board_right = MT9V03X_W;
	result.found_left = false;
	result.found_right = false;

	for (int i = MT9V03X_W / 2; i >= step; i -= step)
	{
		if ((mt9v03x_image[row][i] - mt9v03x_image[row][i - step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i - step]) > threshold)
		{
			result.board_left = i;
			result.found_left = true;
			// printf("l=%d\t", board_left);
			break;
		}
	}
	// printf("|");
	for (int i = MT9V03X_W / 2; i <= MT9V03X_W - 1 - step; i += step)
	{
		if ((mt9v03x_image[row][i] - mt9v03x_image[row][i + step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i + step]) > threshold)
		{
			result.board_right = i;
			result.found_right = true;
			// printf("r=%d\t", board_right);
			break;
		}
	}

	if ((result.found_left || result.found_right) == false)
	{
		// printf("miss");
		for (int i = MT9V03X_W / 2; i <= MT9V03X_W - 1 - step; i += step)
		{
			if ((mt9v03x_image[row][i] - mt9v03x_image[row][i + step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i + step]) < -threshold)
			{
				result.board_left = i;
				result.found_left = true;
				// printf("l=%d\t", found_left);
				break;
			}
		}

		for (int i = MT9V03X_W / 2; i >= step; i -= step)
		{
			if ((mt9v03x_image[row][i] - mt9v03x_image[row][i - step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i - step]) < -threshold)
			{
				result.board_right = i;
				result.found_right = true;
				// printf("r=%d\t", board_right);
				break;
			}
		}
	}

	// for (int i = 0; i <=  MT9V03X_W - 1 - step; i += step)
	// {
	// 	printf("%4d",(mt9v03x_image[row][i] - mt9v03x_image[row][i + step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i + step]));
	// }
	// printf("\n");

	// printf("l=%3d,r=%3d,il=%d,ir=%d\n",result.board_left,result.board_right,result.found_left,result.found_right);
	return result;

}

// **************************** 代码区域 ****************************
