#ifndef _MySerial_h_
#define _MySerial_h_

//��ȡ���ڽ��յ����ַ�������'\0'��'\r'��'\n'��β��
char* getrxs();

//������ڽ��ջ�����
void Clear_rx_buffer();

//�ж��ַ��Ƿ�����������������
int IsChariNum(char*);

//�ж��ַ��Ƿ����ڸ�������������
int IsCharfNum(char*);

/*
��str������ΪWIRELESS_BUFFER_SIZE����Ѱ��subStr�������������num_set��Ϊ������
*/
void GetiValueFromStr(int*, char*, char*);

/*
��str������ΪWIRELESS_BUFFER_SIZE����Ѱ��subStr����ĸ���������num_set��Ϊ�ø�����
*/
void GetfValueFromStr(double*, char*, char*);

#endif