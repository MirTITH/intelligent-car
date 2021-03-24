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
CamResult Camera_Calc_x(int row, int center, int step, int threshold);
int Camera_Find_branch(int row, int col_l, int col_r, int step, int threshold);

int Camera_Calc_y(int col_l, int col_r, int step_x, int step_y, int threshold,  int max_delta_y);

unsigned long long lastT = 0;//上次循环的微秒值
unsigned long long dt = 0;//循环的间隔（微秒）

int midline_40 = MT9V03X_W;  // 其实是左边界+右边界，是中心值的2倍;
// int midline_35;  // 其实是左边界+右边界，是中心值的2倍;
// int midline_30;  // 其实是左边界+右边界，是中心值的2倍;

int CamTurnRate = 270;
// double exp_turnRatio = 0;

int peak_y_pos = -1; //顶点的x坐标

// int branch_turnL = true;

int turnL_Status = 0; //向左转的状态，为0时走中线，小于0时向右走，大于0时向左走

int find_branch = 0;

CamResult cam_result_40;
// CamResult cam_result_35;
// CamResult cam_result_30;

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
		dt = Timer_us_Get() - lastT;
		lastT = Timer_us_Get();

		// PrintData();
		// systick_delay_ms(10);

		if(mt9v03x_finish_flag)
		{
			// TBegin = Timer_us_Get();

			// if (peak_y_pos == -1)
			// {
			// 	cam_result_40 = Camera_Calc_x(40, midline_40 / 2, 5, 12);
			// }
			// else
			// {
			// 	if (branch_turnL)
			// 	{
			// 		cam_result_40 = Camera_Calc_x(40, (peak_y_pos + cam_result_40.board_left) / 2, 5, 12);
			// 	}
			// 	else
			// 	{
			// 		cam_result_40 = Camera_Calc_x(40, (peak_y_pos + cam_result_40.board_right) / 2, 5, 12);
			// 	}
			// }
			
			cam_result_40 = Camera_Calc_x(40, cam_result_40.board_left + (cam_result_40.board_right - cam_result_40.board_left) * (120 - turnL_Status) / 240, 5, 12);

			midline_40 = cam_result_40.board_left + cam_result_40.board_right;

			find_branch = Camera_Find_branch(8, cam_result_40.board_left, cam_result_40.board_right, 5, 18);

			if (find_branch)
			{
				printf("Found\n");
			}
			else
			{
				printf("NOT Found\n");
			}
			
			// peak_y_pos = Camera_Calc_y(cam_result_40.board_left, cam_result_40.board_right, 5, 3, 20, 6);

			// cam_result_35 = Camera_Calc_x(35, midline_40 / 2, 5, 12);
			// midline_35 = cam_result_35.board_left + cam_result_35.board_right;
			// cam_result_30 = Camera_Calc_x(30, midline_35 / 2, 5, 12);
			// midline_30 = cam_result_30.board_left + cam_result_30.board_right;

			if (cam_result_40.found_left && cam_result_40.found_right)
			{
				turnRatio = -(double)(midline_40 - 168 - turnL_Status) / CamTurnRate;
			}
			else if (cam_result_40.found_left && !cam_result_40.found_right)
			{
				turnRatio = -(double)(midline_40 - 168 - turnL_Status) / CamTurnRate;
			}
			else if (!cam_result_40.found_left && cam_result_40.found_right)
			{
				turnRatio = -(double)(midline_40 - 168 - turnL_Status) / CamTurnRate;
			}
			else if (!cam_result_40.found_left && !cam_result_40.found_right)
			{
				turnRatio = (double)turnL_Status / CamTurnRate;
			}

			if (screen_counter > 0)
			{
				// if (peak_y_pos > 0)
				// {
				// 	for (int i = 0; i < MT9V03X_H; i += 2)
				// 	{
				// 		mt9v03x_image[i][peak_y_pos] = 0;
				// 		// mt9v03x_image[i][peak_y_pos - 1] = 0;
				// 		mt9v03x_image[i][midline_40 / 2] = 0;
						
				// 	}
				// }
				ips114_displayimage032(mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
				screen_counter--;
			}
			//seekfree_sendimg_03x(UART_1, mt9v03x_image[0], MT9V03X_W, MT9V03X_H);
			mt9v03x_finish_flag = 0;

			// if (peak_y_pos != -1) printf("py %d\n", peak_y_pos);
			// printf("%lld\n", Timer_us_Get() - TBegin);
		}

		// turnRatio += (exp_turnRatio - turnRatio) * 0.5;

		if (turnRatio > 0.6) turnRatio = 0.6;
		if (turnRatio < -0.6) turnRatio = -0.6;

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
		// printf("m %d,",midline_40);
		// printf("R %llf", turnRatio);
		// printf("bl %d\tbr %d", cam_result_40.board_left, cam_result_40.board_right);
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
		// printf("p %lf,i %lf,d %lf", PID_SC_Kp, PID_SC_Ki, PID_SC_Kd);
		// printf(",ap %lf,ai %lf,ad %lf", AngleControl_P, AngleControl_I, AngleControl_D);
		// printf(",ab %lf", exp_acc_angle_yz);
		// printf(",ang %lf", angle);
		// printf(",cp %lf,cd %lf", AC_CarSpeed_P, AC_CarSpeed_D);
		// printf(",ct %d", CamTurnRate);
		// printf(",tl %d", turnL_Status);
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
	GetiValueFromStr(&turnL_Status, str, "tl=");
	// GetiValueFromStr(&branch_turnL, str, "tl=");

	//printf("%s", str);

}

CamResult Camera_Calc_x(int row, int center, int step, int threshold)
{
	CamResult result;
	result.board_left = 0;
	result.board_right = MT9V03X_W - 1;
	result.found_left = false;
	result.found_right = false;

	for (int i = center; i >= step; i -= step)
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
	for (int i = center; i <= MT9V03X_W - 1 - step; i += step)
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
		for (int i = center; i <= MT9V03X_W - 1 - step; i += step)
		{
			if ((mt9v03x_image[row][i] - mt9v03x_image[row][i + step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i + step]) < -threshold)
			{
				result.board_left = i;
				result.found_left = true;
				// printf("l=%d\t", found_left);
				break;
			}
		}

		for (int i = center; i >= step; i -= step)
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

int Camera_Calc_y(int col_l, int col_r, int step_x, int step_y, int threshold, int max_delta_y)
{
	int last_y = -1;
	int peak_x; //顶点的x坐标
	int peak_y = -1;//顶点的y坐标

	for (int x = col_l; x <= col_r; x += step_x)
	{
		for (int y = MT9V03X_H - 1; y - step_y>= 0; y -= step_y)
		{
			if((mt9v03x_image[y][x] - mt9v03x_image[y - step_y][x]) * 100 / (mt9v03x_image[y][x] +mt9v03x_image[y - step_y][x]) > threshold)
			{
				if (y > last_y)
				{
					last_y = y;
					peak_x = x;
					peak_y = y;
				}
				else
				{
					if (peak_y - y > max_delta_y)
					{

						return peak_x;
					}
				}
				
			}
		}
	}

	return -1;
}

int Camera_Find_branch(int row, int col_l, int col_r, int step, int threshold)
{
	int left = 0;
	int right = 0;
	for (int i = col_l; i <= col_r; i += step)
	{
		if (i + step > MT9V03X_W - 1) break;

		if ((mt9v03x_image[row][i] - mt9v03x_image[row][i + step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i + step]) > threshold)
		{
			left = i;
			break;
		}
	}
	// printf("|");
	for (int i = col_r; i >= col_l; i -= step)
	{
		if (i - step < 0) break;

		if ((mt9v03x_image[row][i] - mt9v03x_image[row][i - step]) * 100 / (mt9v03x_image[row][i] + mt9v03x_image[row][i - step]) > threshold)
		{
			right = i;
			break;
		}
	}

	if (right - left > 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// **************************** 代码区域 ****************************
