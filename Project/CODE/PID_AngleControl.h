#ifndef _PID_AngleControl_h_
#define _PID_AngleControl_h_

extern bool PID_AngleControl_On;
extern double angle_yz_err;


void PID_AngleControl_init();
void Angle_Set();
void PID_AngleControl_Calc();
void Update_Gyro_Acc();

#endif