#include "headfile.h"
#include "PID_SpeedControl.h"
#include "PID_AngleControl.h"

#define PID_AngleControl_Calc_Feq 1000
#define MAX_bal_acc_angle_yz 0.91
#define MIN_bal_acc_angle_yz 0.75

const double PI = 3.1415926535897932384626433832795;

bool PID_AngleControl_On = false;
double acc_angle_yz = 0;//y,z轴方向加速度的角度
double bal_acc_angle_yz = 0.83;//平衡时的角度
double angle_yz_err = 0;

double AngleControl_P = 500;
double AngleControl_I = 500;
double AngleControl_D = 0;//暂未使用

double AC_CarSpeed_P = 0;
double AC_CarSpeed_D = 0;

double PID_AC_I_Value = 0;

double speed_car = 0;//期望车速


double Motor_AngleControl_Speed = 0;
//double Motor2_AngleControl_Speed = 0;

// double angle;                //数据融合后的角度    

double acc_ratio = 1;      //加速度计比例    

double gyro_ratio = 0.001;    //陀螺仪比例

void PID_AngleControl_init()
{
	tim_interrupt_init(TIM_16, PID_AngleControl_Calc_Feq, 2);
	PID_AngleControl_On = true;
}

void PID_AngleControl_Calc()
{
	Update_Gyro_Acc();
	static double last_sE = 0;
	static double sE = 0;
	if (PID_AngleControl_On)
	{
		PID_AC_I_Value += AngleControl_I * angle_yz_err / PID_AngleControl_Calc_Feq;
		Motor_AngleControl_Speed = AngleControl_P * angle_yz_err + PID_AC_I_Value;
		exp_Speed1 = Motor_AngleControl_Speed;
		exp_Speed2 = Motor_AngleControl_Speed;

		sE = exp_Speed1 - speed_car;
		bal_acc_angle_yz += sE * AC_CarSpeed_P / PID_AngleControl_Calc_Feq + (sE - last_sE) * AC_CarSpeed_D;
		if (bal_acc_angle_yz > MAX_bal_acc_angle_yz) bal_acc_angle_yz = MAX_bal_acc_angle_yz;
		if (bal_acc_angle_yz < MIN_bal_acc_angle_yz) bal_acc_angle_yz = MIN_bal_acc_angle_yz;

		last_sE = sE;
	}
}

//更新加速度、角度值
void Update_Gyro_Acc()
{
		get_icm20602_gyro_spi();
		get_icm20602_accdata_spi();

		if (icm_acc_z < 0)
		{
			acc_angle_yz = atan((double)icm_acc_y / icm_acc_z);
		}
		else if (icm_acc_z > 0)
		{
			acc_angle_yz = atan((double)icm_acc_y / icm_acc_z) + PI;
		}
		else
		{
			acc_angle_yz = PI / 2;
		}
		angle_yz_err = bal_acc_angle_yz - angle_calc(acc_angle_yz, (double)icm_gyro_x);
}

double angle_calc(double angle_m, double gyro_m)

{    

//    double temp_angle;

    double gyro_now;

    double error_angle;    



    static double last_angle;

	// static double last_angle_m;    

    static uint8 first_angle;    

        

    if(!first_angle)//判断是否为第一次运行本函数    

    {    

        //如果是第一次运行，则将上次角度值设置为与加速度值一致    

        first_angle = 1;    

        last_angle = angle_m;    

    }    

        

    gyro_now = gyro_m * gyro_ratio;

      

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差    

    error_angle = (angle_m - last_angle)*acc_ratio;  

    // printf("g %lf,a %lf\n", gyro_m, angle_m - last_angle_m);
	// last_angle_m = angle_m;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值    

    last_angle += (error_angle + gyro_now)/PID_AngleControl_Calc_Feq;


    return last_angle;    

}
