#include "stm32f0xx.h"
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_misc.h>
#include <stdbool.h>

#define TIMER_PERIOD 2000
#define TIMER_PRESCALER 480
#define PWM_PULSE_LENGHT(val) {TIM3->CCR1 = val;}

/*
 * GPIO
 */
void initializeDiscoveryLeds(void);
void initializeUserButton(void);
/*
 * PWM
 */
void InitializeTimer(void);
void InitializePwmPin(void);
void InitializePwm(void);

bool throttleLow = false;

int main(void)
{
	initializeDiscoveryLeds();
	initializeUserButton();
	InitializeTimer();
	InitializePwmPin();
	InitializePwm();
	GPIO_SetBits(GPIOC,GPIO_Pin_9);

	for(;;){
	}
}

/*
 * Interrupts
 */

void EXTI0_1_IRQHandler(void)					//External interrupt handlers
{
	if(EXTI_GetITStatus(EXTI_Line0) == SET){	//Handler for Button2 pin interrupt

		throttleLow = !throttleLow;

		if(throttleLow){
			GPIO_SetBits(GPIOC,GPIO_Pin_8);
			PWM_PULSE_LENGHT(85);
		}
		else{
			GPIO_ResetBits(GPIOC,GPIO_Pin_8);
			PWM_PULSE_LENGHT(110);
		}
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}


/*
 * Functions
 */
void initializeDiscoveryLeds(void){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_structure;
	GPIO_structure.GPIO_Pin=GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_structure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_structure.GPIO_OType = GPIO_OType_PP;
	GPIO_structure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_structure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOC, &GPIO_structure);
}

void initializeUserButton(void){
	//Interrupt pin - PA8

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_structure;
	GPIO_structure.GPIO_Pin=GPIO_Pin_0;
	GPIO_structure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_structure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_structure);

	RCC_APB2PeriphClockCmd(RCC_APB2ENR_SYSCFGEN, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

	EXTI_InitTypeDef   EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line= EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitTypeDef   NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0F;
	NVIC_Init(&NVIC_InitStructure);
}

void InitializePwm()
{
	TIM_OCInitTypeDef  TIM_OCInitStructure ;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 240;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	//TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	//TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	//TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_1);
}

void InitializeTimer(void)
{
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	  /* Time Base configuration */
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_TimeBaseStructure.TIM_Prescaler = TIMER_PRESCALER;
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	  TIM_TimeBaseStructure.TIM_Period = TIMER_PERIOD;
	  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	  TIM_Cmd(TIM3, ENABLE);
}

void InitializePwmPin()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


