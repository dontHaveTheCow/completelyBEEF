#include "accTimer.h"

void Initialize_accTimer(void){

	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM14EN, ENABLE);
	TIM_TimeBaseInitTypeDef Timer_init_structure;
	Timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;
	Timer_init_structure.TIM_ClockDivision = 0;
	Timer_init_structure.TIM_Prescaler = TIM14_PRESCALER_VALUE;
	Timer_init_structure.TIM_Period = TIM14_PERIOD_VALUE;
	Timer_init_structure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM14, &Timer_init_structure);

}

void accTimer_interrupt_enable(void){

	NVIC_InitTypeDef NVIC_structure;
	NVIC_structure.NVIC_IRQChannel = TIM14_IRQn;
	NVIC_structure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_structure.NVIC_IRQChannelPriority = 0x01;
	NVIC_Init(&NVIC_structure);
	TIM_Cmd(TIM14,DISABLE);
	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
}


