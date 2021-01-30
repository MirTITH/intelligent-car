#include "headfile.h"
#include "Timer_us.h"

unsigned long long TimeCountPeriod = 0;


//使用TIM14中断计时
void Timer_us_init()
{
	TimeCountPeriod = 0;
	RCC->APB2ENR |= ((uint32_t)0x00000001 << 16);//TIM14 定时器使能
	TIM14->ARR = 0xffff;
	TIM14->PSC = 95;
	TIM14->DIER |= TIM_DIER_UI;

	//中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM14->CR1 |= TIM_CR1_CEN;
}

unsigned long long Timer_us_Get()
{
	return TimeCountPeriod * (unsigned long long)65535 + TIM14->CNT;
}