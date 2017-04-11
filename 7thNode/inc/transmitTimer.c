#include "transmitTimer.h"

void Initialize_transmitTimer(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_TIM15EN, ENABLE);
	TIM_TimeBaseInitTypeDef Timer_init_structure;
	Timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
	Timer_init_structure.TIM_ClockDivision = 0;
	Timer_init_structure.TIM_Prescaler = TIM15_PRESCALER_VALUE;
	Timer_init_structure.TIM_Period = TIM15_PERIOD_VALUE;
	Timer_init_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM15, &Timer_init_structure);

	TIM_Cmd(TIM15,ENABLE);

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}

void transmitTimer_interrupt_enable(void)
{
	NVIC_InitTypeDef NVIC_structure;
	NVIC_structure.NVIC_IRQChannel = TIM15_IRQn;
	NVIC_structure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_structure.NVIC_IRQChannelPriority = 0x00;
	NVIC_Init(&NVIC_structure);
	TIM_ITConfig(TIM15, TIM_IT_Update, DISABLE);
}



