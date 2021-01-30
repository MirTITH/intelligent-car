# 高级控制定时器
## TIM1
中断测速（软件）

## TIM8


# 32位通用定时器
## TIM2
电机PWM输出

# 16位通用定时器
## TIM3
软件中断，PID电机电压计算


# 基本定时器
## TIM14
微秒级计时

# 基本定时器
## TIM16
软件中断，PID角度控制
## TIM17

# 中断优先级
Project\CODE\PID_SpeedControl.c
``` c
tim_interrupt_init(TIM_1, SpeedConut_Feq, 1);//测速
tim_interrupt_init(TIM_3, PID_SpeedControl_Calc_Feq, 2); //用于PID计算
exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//编码器
exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//编码器
```

Project\CODE\Timer_us.c

- TIM14中断
	- 微秒级计时
	- 优先级 3