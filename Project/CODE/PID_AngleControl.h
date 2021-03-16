#ifndef _PID_AngleControl_h_
#define _PID_AngleControl_h_


extern bool PID_AngleControl_On;
extern double angle_yz_err;
extern double bal_acc_angle_yz;
extern double angle; //数据融合后的角度
extern const double PI;

//角度姿态PID控制比例系数
extern double AngleControl_P;

//角度姿态PID控制积分系数
extern double AngleControl_I;

//角度姿态PID控制微分系数
extern double AngleControl_D;

extern double AC_CarSpeed_P; //期望角度控制比例项
extern double AC_CarSpeed_D; //期望角度控制微分项

extern double PID_AC_I_Value;

extern double acc_ratio;
extern double gyro_ratio;

//期望车速
extern double car_speed;
//转弯系数（左转弯为正）
extern double turnRatio;
// extern double acc_angle_yz;

void PID_AngleControl_init();
void PID_AngleControl_Calc();
//void PID_balAngle_Calc();
void Update_Gyro_Acc();
double angle_calc();


#endif