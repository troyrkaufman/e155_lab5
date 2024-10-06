// STM32F401RE_TIM.h
// Header for TIM functions

#ifndef STM32L4_TIM_H
#define STM32L4_TIM_H

#include <stdint.h> // Include stdint header
#include <stm32l432xx.h>
#include "STM32L432KC_GPIO.h"

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void pwm_init(TIM_TypeDef * TIMx);
void pwm_update(TIM_TypeDef * TIMx, uint32_t freq);
void delay_init(TIM_TypeDef * TIMx);
void delay_update(TIM_TypeDef * TIMx, uint32_t duration);
#endif