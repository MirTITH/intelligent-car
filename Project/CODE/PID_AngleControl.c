#include "headfile.h"
#include "PID_SpeedControl.h"
#include "PID_AngleControl.h"

#define PID_AngleControl_Calc_Feq 200
#define MAX_exp_acc_angle_yz 1.00
#define MIN_exp_acc_angle_yz 0.75

const double PI = 3.1415926535897932384626433832795;

bool PID_AngleControl_On = false;
double acc_angle_yz = 0;//y,z轴方向加速度的角度
double exp_acc_angle_yz = 0.903;
double last_angle_yz_err;
double angle_yz_err = 0;

// double AngleControl_P = 300;
// double AngleControl_I = 1000;
// double AngleControl_D = 0;

double AngleControl_P = 300;
double AngleControl_I = 1000;
double AngleControl_D = 3;

double AC_CarSpeed_P = 0.0013;
double AC_CarSpeed_D = 0.001;

double PID_AC_I_Value = 0;//积分项的值
double PID_AC_D_Value = 0;//微分项的值
double Max_PID_AC_D_Value = 100;

double car_speed = 0;
double turnRatio = 0; 

double Motor_AngleControl_Speed = 0;
//double Motor2_AngleControl_Speed = 0;

double angle = 0;

double acc_ratio = 0.005;      //加速度计比例    

double gyro_ratio = 0.001;    //陀螺仪比例

int duoji = 0;

void PID_AngleControl_init()
{
	systick_delay_ms(300);
	tim_interrupt_init(TIM_16, PID_AngleControl_Calc_Feq, 2);
	PID_AngleControl_On = true;
}

void PID_AngleControl_Calc()
{
	Update_Gyro_Acc();
	static double last_sE = 0;
	static double sE = 0;//当前车速与期望车速的差

	duoji = (1.5 - angle) * 5000 / PI + 1250;
	if (duoji < 1250) duoji = 1250;
	if (duoji > 6250) duoji = 6250;
	pwm_duty_updata(TIM_17, TIM_17_CH1_A07, duoji);

	if (PID_AngleControl_On)
	{
		PID_AC_I_Value += AngleControl_I * angle_yz_err / PID_AngleControl_Calc_Feq;
		PID_AC_D_Value = (angle_yz_err - last_angle_yz_err) * PID_AngleControl_Calc_Feq * AngleControl_D;

		if (PID_AC_D_Value > Max_PID_AC_D_Value) PID_AC_D_Value = Max_PID_AC_D_Value;
		if (PID_AC_D_Value < -Max_PID_AC_D_Value) PID_AC_D_Value = -Max_PID_AC_D_Value;

		Motor_AngleControl_Speed = AngleControl_P * angle_yz_err + PID_AC_I_Value + PID_AC_D_Value;
		exp_Speed1 = Motor_AngleControl_Speed * (1 - turnRatio);
		exp_Speed2 = Motor_AngleControl_Speed * (1 + turnRatio);

		sE = car_speed - Motor_AngleControl_Speed;
		exp_acc_angle_yz -= sE * AC_CarSpeed_P / PID_AngleControl_Calc_Feq + (sE - last_sE) * AC_CarSpeed_D;
		if (exp_acc_angle_yz > MAX_exp_acc_angle_yz) exp_acc_angle_yz = MAX_exp_acc_angle_yz;
		if (exp_acc_angle_yz < MIN_exp_acc_angle_yz) exp_acc_angle_yz = MIN_exp_acc_angle_yz;

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
		last_angle_yz_err = angle_yz_err;
		angle = angle_calc(acc_angle_yz, (double)icm_gyro_x);
		angle_yz_err = exp_acc_angle_yz - angle;
}

double angle_calc(double angle_m, double gyro_m)

{    

//    double temp_angle;

    double gyro_now;

    double error_angle;    



    static double last_angle;

	// static double last_angle_m;    

    static bool first_angle;    

        

    if(!first_angle)//判断是否为第一次运行本函数    

    {    

        //如果是第一次运行，则将上次角度值设置为与加速度值一致    

        first_angle = true;    

        last_angle = angle_m;    

    }    

        

    gyro_now = gyro_m * gyro_ratio;

      

    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差    

    error_angle = (angle_m - last_angle)*acc_ratio;  

    // printf("g %lf,a %lf\n", gyro_m, angle_m - last_angle_m);
	// last_angle_m = angle_m;

    //根据偏差与陀螺仪测量得到的角度值计算当前角度值    

    last_angle += error_angle + gyro_now / PID_AngleControl_Calc_Feq;


    return last_angle;    

}

