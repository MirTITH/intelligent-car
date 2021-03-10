#ifndef _MySerial_h_
#define _MySerial_h_

//获取串口接收到的字符串（以'\0'或'\r'或'\n'结尾）
char* getrxs();

//清除串口接收缓冲区
void Clear_rx_buffer();

//判断字符是否属于整数或正负号
int IsChariNum(char*);

//判断字符是否属于浮点数或正负号
int IsCharfNum(char*);

/*
从str（长度为WIRELESS_BUFFER_SIZE）中寻找subStr后面的整数并将num_set设为该整数
*/
void GetiValueFromStr(int*, char*, char*);

/*
从str（长度为WIRELESS_BUFFER_SIZE）中寻找subStr后面的浮点数并将num_set设为该浮点数
*/
void GetfValueFromStr(double*, char*, char*);

#endif