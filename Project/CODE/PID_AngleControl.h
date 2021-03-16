#ifndef _PID_AngleControl_h_
#define _PID_AngleControl_h_


extern bool PID_AngleControl_On;
extern double angle_yz_err;
extern double bal_acc_angle_yz;
extern double angle; //�����ںϺ�ĽǶ�
extern const double PI;

//�Ƕ���̬PID���Ʊ���ϵ��
extern double AngleControl_P;

//�Ƕ���̬PID���ƻ���ϵ��
extern double AngleControl_I;

//�Ƕ���̬PID����΢��ϵ��
extern double AngleControl_D;

extern double AC_CarSpeed_P; //�����Ƕȿ��Ʊ�����
extern double AC_CarSpeed_D; //�����Ƕȿ���΢����

extern double PID_AC_I_Value;

extern double acc_ratio;
extern double gyro_ratio;

//��������
extern double car_speed;
//ת��ϵ������ת��Ϊ����
extern double turnRatio;
// extern double acc_angle_yz;

void PID_AngleControl_init();
void PID_AngleControl_Calc();
//void PID_balAngle_Calc();
void Update_Gyro_Acc();
double angle_calc();


#endif