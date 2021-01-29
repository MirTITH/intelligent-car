# TIM

## 基本定时器TIM14

- 16位计数器
- 分频系数1~65536
- 只能向上计数
- 接在APB2上

### 计时方法
``` c
RCC->APB2ENR?|=?((uint32_t)0x00000001?<<?16);//TIM14?定时器使能
TIM14->ARR?=?0xffff;//自动重装载值设为65535
TIM14->PSC?=?0xffff;//预分频值设为65535
TIM14->CR1?|=?TIM_CR1_CEN;//启动TIM14
```

### 中断方法
``` c
RCC->APB2ENR |= ((uint32_t)0x00000001 << 16);//TIM14 定时器使能
//ARR和PSC这样设置后周期就是1s
TIM14->ARR = 32764;
TIM14->PSC = 2929;

TIM14->DIER |= TIM_DIER_UI;//中断使能（打开了才能中断）

//中断配置
NVIC_InitTypeDef NVIC_InitStructure;//定义一个结构体，用于配置中断参数
NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;//使用TIM14中断
NVIC_InitStructure.NVIC_IRQChannelPriority = 0;//中断优先级[0,3]
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//启动中断

NVIC_Init(&NVIC_InitStructure);//中断配置函数

TIM14->CR1 |= TIM_CR1_CEN;//启动TIM14
```