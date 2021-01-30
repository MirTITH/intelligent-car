#include "headfile.h"
#include "PID_AngleControl.h"

#define PID_AngleControl_Calc_Feq 100

bool PID_AngleControl_On = false;
double acc_angle_yz = 0;//y,z轴方向加速度的角度
double bal_acc_angle_yz = 0.81;//平衡时的角度
double angle_yz_err = 0;

double AngleControl_P = 10000;
double AngleControl_I = 10000;
double AngleControl_D = 0;


double Motor1_AngleControl_Speed = 0;
double Motor2_AngleControl_Speed = 0;


void PID_AngleControl_init()
{
	tim_interrupt_init(TIM_16, PID_AngleControl_Calc_Feq, 2);
	PID_AngleControl_On = true;
}

void PID_AngleControl_Calc()
{
	if (PID_AngleControl_On)
	{
		static double I_Value = 0;
		Update_Gyro_Acc();
		I_Value += AngleControl_I * angle_yz_err / PID_AngleControl_Calc_Feq;
		Motor1_AngleControl_Speed = AngleControl_P * angle_yz_err + I_Value;
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
		angle_yz_err = acc_angle_yz - bal_acc_angle_yz;
}