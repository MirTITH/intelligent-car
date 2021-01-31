#ifndef _PID_AngleControl_h_
#define _PID_AngleControl_h_

extern bool PID_AngleControl_On;
extern double angle_yz_err;
extern double bal_acc_angle_yz;
extern const double PI;

extern double AngleControl_P;
extern double AngleControl_I;
extern double AngleControl_D;

extern double I_Value;

void PID_AngleControl_init();
void Angle_Set();
void PID_AngleControl_Calc();
void Update_Gyro_Acc();

#endif