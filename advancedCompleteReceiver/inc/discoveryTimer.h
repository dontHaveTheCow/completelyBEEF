#ifndef DISCOVERYTIMER_H_
#define DISCOVERYTIMER_H_

//These are the Includes
#include <stm32f0xx.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_misc.h>

//These are the Define statements
#define TIM3_PRESCALER_VALUE 48000
#define TIM3_PERIOD_VALUE 13000

//Prescaler 4800 and period 150 corresponds to 15ms

//These are the prototypes for the routines
void Initialize_discTimer(void);
void discTimer_interrupt_enable(void);

#endif /* DISCOVERYTIMER_H_ */
