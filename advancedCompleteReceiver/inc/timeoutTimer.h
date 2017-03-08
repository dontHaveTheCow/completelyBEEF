/*
 * transmitTimer.h
 *
 *  Created on: Jan 12, 2017
 *      Author: niks
 */

#ifndef TIMEOUTTIMER_H_
#define TIMEOUTTIMER_H_

//These are the Includes
#include <stm32f0xx.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_misc.h>

//These are the Define statements
#define TIM15_PRESCALER_VALUE 48000 //1 tick equals 1ms
#define TIM15_PERIOD_VALUE 900

//These are the prototypes for the routines
void Initialize_timeoutTimer(void);
void timeoutTimer_interrupt_enable(void);

#endif /* TIMEOUTTIMER_H_ */
