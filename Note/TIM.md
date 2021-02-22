# TIM

## ������ʱ��TIM14

- 16λ������
- ��Ƶϵ��1~65536
- ֻ�����ϼ���
- ����APB2��

### ��ʱ����
``` c
RCC->APB2ENR?|=?((uint32_t)0x00000001?<<?16);//TIM14?��ʱ��ʹ��
TIM14->ARR?=?0xffff;//�Զ���װ��ֵ��Ϊ65535
TIM14->PSC?=?0xffff;//Ԥ��Ƶֵ��Ϊ65535
TIM14->CR1?|=?TIM_CR1_CEN;//����TIM14
```

### �жϷ���
``` c
RCC->APB2ENR |= ((uint32_t)0x00000001 << 16);//TIM14 ��ʱ��ʹ��
//ARR��PSC�������ú����ھ���1s
TIM14->ARR = 32764;
TIM14->PSC = 2929;

TIM14->DIER |= TIM_DIER_UI;//�ж�ʹ�ܣ����˲����жϣ�

//�ж�����
NVIC_InitTypeDef NVIC_InitStructure;//����һ���ṹ�壬���������жϲ���
NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;//ʹ��TIM14�ж�
NVIC_InitStructure.NVIC_IRQChannelPriority = 0;//�ж����ȼ�[0,3]
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//�����ж�

NVIC_Init(&NVIC_InitStructure);//�ж����ú���

TIM14->CR1 |= TIM_CR1_CEN;//����TIM14
```