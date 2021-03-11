#include "headfile.h"
#include "MySerial.h"
#include <stdlib.h>

char* getrxs()
{
	if (wireless_rx_index == 0)
	{
		if (!(wireless_rx_buffer[WIRELESS_BUFFER_SIZE - 1] == '\r' || wireless_rx_buffer[WIRELESS_BUFFER_SIZE - 1] == '\n' || wireless_rx_buffer[WIRELESS_BUFFER_SIZE - 1] == '\0'))
		{
			return NULL;
		}
	}
	else
	{
		if (!(wireless_rx_buffer[wireless_rx_index - 1] == '\r' || wireless_rx_buffer[wireless_rx_index - 1] == '\n' || wireless_rx_buffer[wireless_rx_index - 1] == '\0'))
		{
			return NULL;
		}
	}
	
	int buffer_cursor = wireless_rx_index;
	bool circulate_flag = false;
	static char result[WIRELESS_BUFFER_SIZE + 1] = {0};
	buffer_cursor -= 2;
	if (buffer_cursor < 0)
	{
		buffer_cursor += WIRELESS_BUFFER_SIZE;
	}

	while (!(wireless_rx_buffer[buffer_cursor] == '\r' || wireless_rx_buffer[buffer_cursor] == '\0' || wireless_rx_buffer[buffer_cursor] == '\n'))
	{
		buffer_cursor--;

		if (buffer_cursor < 0)
		{
			if (circulate_flag == true)
			{
				return NULL;
			}
			circulate_flag = true;
			buffer_cursor += WIRELESS_BUFFER_SIZE;
		}
	}

	for (int i = 0; i <= WIRELESS_BUFFER_SIZE; i++)
	{
		buffer_cursor++;
		if (buffer_cursor >= WIRELESS_BUFFER_SIZE)
		{
			buffer_cursor -= WIRELESS_BUFFER_SIZE;
		}

		if (buffer_cursor == wireless_rx_index)
		{
			result[i] = '\0';
			break;
		}
		result[i] = wireless_rx_buffer[buffer_cursor];
	}

	Clear_rx_buffer();
	return result;
}

void Clear_rx_buffer()
{
	if (wireless_rx_index - 2 < 0)
	{
		wireless_rx_buffer[WIRELESS_BUFFER_SIZE + wireless_rx_index - 2] = '\0';
	}
	else
	{
		wireless_rx_buffer[wireless_rx_index - 2] = '\0';
	}
	
}

int IsChariNum(char* cha)
{
	return ((*cha >= '0' && *cha <= '9') || *cha == '-' || *cha == '+');
}

int IsCharfNum(char* cha)
{
	return ((*cha >= '0' && *cha <= '9') || *cha == '-' || *cha == '+' || *cha == '.');
}

void GetiValueFromStr(int* num_set, char* str, char* subStr)
{
	int num_end = 0;
	char* str_pointer = 0;
	char str_num[WIRELESS_BUFFER_SIZE] = {0};

	str_pointer = strstr(str, subStr);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(subStr);
		num_end = 0;
		while (IsChariNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';
		*num_set = atoi(str_num);
	}
}

void GetfValueFromStr(double* num_set, char* str, char* subStr)
{
	int num_end = 0;
	char* str_pointer = 0;
	char str_num[WIRELESS_BUFFER_SIZE] = {0};

	str_pointer = strstr(str, subStr);
	if (str_pointer != NULL)
	{
		str_pointer += strlen(subStr);
		num_end = 0;
		while (IsCharfNum(str_pointer + num_end))
		{
			num_end++;
		}

		for (int i = 0; i < num_end; i++)
		{
			str_num[i] = str_pointer[i];
		}
		str_num[num_end] = '\0';
		*num_set = atof(str_num);
	}
}
