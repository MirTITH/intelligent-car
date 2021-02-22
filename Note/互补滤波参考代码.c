#include "headfile.h"    

    

float angle;                //�����ںϺ�ĽǶ�    

    

float acc_ratio = 0.6;      //���ٶȼƱ���    

float gyro_ratio = 0.38;    //�����Ǳ���    

float dt = 0.005;           //��������    

    

//----------------------------------------------------------------    

//  @brief      һ�׻����˲�    

//  @param      angle_m     ���ٶȼ�����    

//  @param      gyro_m      ����������    

//  @return     float       �����ںϺ�ĽǶ�    

//----------------------------------------------------------------    

float angle_calc(float angle_m, float gyro_m)    

{    

    float temp_angle;               

    float gyro_now;    

    float error_angle;    

        

    static float last_angle;    

    static uint8 first_angle;    

        

    if(!first_angle)//�ж��Ƿ�Ϊ��һ�����б�����    

    {    

        //����ǵ�һ�����У����ϴνǶ�ֵ����Ϊ����ٶ�ֵһ��    

        first_angle = 1;    

        last_angle = angle_m;    

    }    

        

    gyro_now = gyro_m * gyro_ratio;    

      

    //���ݲ������ļ��ٶ�ֵת��Ϊ�Ƕ�֮�����ϴεĽǶ�ֵ��ƫ��    

    error_angle = (angle_m - last_angle)*acc_ratio;  

      

    //����ƫ���������ǲ����õ��ĽǶ�ֵ���㵱ǰ�Ƕ�ֵ    

    temp_angle = last_angle + (error_angle + gyro_now)*dt;   

      

    //���浱ǰ�Ƕ�ֵ    

    last_angle = temp_angle;  

      

    return temp_angle;    

}    

    

int main(void)    

{    

    uint8 virsco_data[10];                

        

    DisableGlobalIRQ();    

    board_init();//��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���    

      

    systick_delay_ms(300);  

      

    icm20602_init_spi();        //���������ǳ�ʼ��    

    seekfree_wireless_init();   //����ת���ڳ�ʼ��    

    

    systick_delay_ms(50);       //��ʼ����ɺ���ʱһ����ʱ��  

    //���ж������    

    EnableGlobalIRQ(0);    

    while (1)    

    {    

        get_icm20602_accdata_spi();     //��ȡ���ٶ�����    

        get_icm20602_gyro_spi();        //��ȡ����������    

            

        angle = angle_calc(icm_acc_z, icm_gyro_y); //һ�׻����˲�����Ƕ�ֵ  

            

        data_conversion(icm_acc_z, icm_gyro_y, (int)angle, 0, virsco_data); //�������ݵ���λ��    

        seekfree_wireless_send_buff(virsco_data, 10);    

            

        systick_delay_ms(5);//��ʱ5ms    

    }    

} 