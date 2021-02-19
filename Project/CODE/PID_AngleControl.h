#ifndef _PID_AngleControl_h_
#define _PID_AngleControl_h_

extern bool PID_AngleControl_On;
extern double angle_yz_err;
extern double bal_acc_angle_yz;
extern const double PI;

extern double AngleControl_P;
extern double AngleControl_I;
extern double AngleControl_D;

extern double AC_CarSpeed_P;
extern double AC_CarSpeed_D;

extern double PID_AC_I_Value;

extern double acc_ratio;
extern double gyro_ratio;

extern double speed_car;
extern double turnRatio;
// extern double acc_angle_yz;

void PID_AngleControl_init();
void PID_AngleControl_Calc();
//void PID_balAngle_Calc();
void Update_Gyro_Acc();
double angle_calc(angle_m, gyro_m);


#endif