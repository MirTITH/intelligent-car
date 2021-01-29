# TIM

## 基本定时器TIM14

- 16位计数器
- 分频系数1~65536
- 只能向上计数
- 接在APB2上
``` c
RCC->APB2ENR?|=?((uint32_t)0x00000001?<<?16);//TIM14?定时器使能
TIM14->ARR?=?0xffff;//自动重装载值设为65535
TIM14->PSC?=?0xffff;//预分频值设为65535
TIM14->CR1?|=?TIM_CR1_CEN;//启动TIM14
```