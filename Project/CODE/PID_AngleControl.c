#include "headfile.h"
#include "PID_SpeedControl.h"
#include "PID_AngleControl.h"

#define PID_AngleControl_Calc_Feq 1000

const double PI = 3.1415926535897932384626433832795;

bool PID_AngleControl_On = false;
double acc_angle_yz = 0;//y,z轴方向加速度的角度
double bal_acc_angle_yz = 0.823;//平衡时的角度
double angle_yz_err = 0;

double AngleControl_P = 350;
double AngleControl_I = 0;
double AngleControl_D = 0;

double PID_AC_I_Value = 0;


double Motor_AngleControl_Speed = 0;
//double Motor2_AngleControl_Speed = 0;


void PID_AngleControl_init()
{
	tim_interrupt_init(TIM_16, PID_AngleControl_Calc_Feq, 2);
	PID_AngleControl_On = true;
}

void PID_AngleControl_Calc()
{
	Update_Gyro_Acc();
	if (PID_AngleControl_On)
	{
		PID_AC_I_Value += AngleControl_I * angle_yz_err / PID_AngleControl_Calc_Feq;
		Motor_AngleControl_Speed = AngleControl_P * angle_yz_err + PID_AC_I_Value;
		exp_Speed1 = Motor_AngleControl_Speed;
		exp_Speed2 = Motor_AngleControl_Speed;
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
		angle_yz_err = bal_acc_angle_yz - acc_angle_yz;
}