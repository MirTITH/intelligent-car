# �߼����ƶ�ʱ��
## TIM1
�жϲ��٣������

## TIM8


# 32λͨ�ö�ʱ��
## TIM2
���PWM���

# 16λͨ�ö�ʱ��
## TIM3
����жϣ�PID�����ѹ����


# ������ʱ��
## TIM14
΢�뼶��ʱ

# ������ʱ��
## TIM16
����жϣ�PID�Ƕȿ���
## TIM17

# �ж����ȼ�
Project\CODE\PID_SpeedControl.c
``` c
tim_interrupt_init(TIM_1, SpeedConut_Feq, 1);//����
tim_interrupt_init(TIM_3, PID_SpeedControl_Calc_Feq, 2); //����PID����
exti_interrupt_init(C0, EXTI_Trigger_Rising_Falling, 0x00);//������
exti_interrupt_init(A1, EXTI_Trigger_Rising_Falling, 0x00);//������
```

Project\CODE\Timer_us.c

- TIM14�ж�
	- ΢�뼶��ʱ
	- ���ȼ� 3