/*
 * adcTimer.h
 *
 *  Created on: Dec 28, 2016
 *      Author: niks
 */

#ifndef ACCTIMER_H_
#define ACCTIMER_H_

//These are the Includes
#include <stm32f0xx.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_misc.h>

//These are the Define statements
#define TIM14_PRESCALER_VALUE 4800
#define TIM14_PERIOD_VALUE 200

//Prescaler 4800 and period 150 corresponds to 20ms

//These are the prototypes for the routines
void Initialize_accTimer(void);
void accTimer_interrupt_enable(void);

#endif /* ACCTIMER_H_ */



