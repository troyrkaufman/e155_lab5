// STM32F401RE_TIM.c
// TIM functions

#include "STM32L432KC_TIM.h"
#include "STM32L432KC_RCC.h"
#include <stm32l432xx.h>

extern volatile int start_count;

void count_init(TIM_TypeDef * TIMx){
   TIMx->PSC = 799;                   // Sets CK_CNT to 100 KHz (Sysclock is 80 MHz) to avoid overflowing counter
   TIMx->CR1 |= (1<<7);               // Auto-reload preload enabled
   TIMx->EGR |= (1<<0);               // Initialize all registers to allow preload registers
   TIMx->CR1 |= (1<<0);               // Start TIMx counter
}

void delay_init(TIM_TypeDef * TIMx){
   TIMx->PSC = 799;                  // Sets CK_CNT to 10KHz (Sysclock is 80 MHz)
   TIMx->CR1 |= (1<<7);               // Auto-reload preload enabled
   TIMx->EGR |= (1<<0);               // Initialize all registers to allow preload registers
   TIMx->CR1 |= (1<<0);               // Start TIMx counter
}

void delay_update(TIM_TypeDef * TIMx, uint32_t duration){
   TIMx->ARR = (duration * 10) - 1;            // With a 10KHz clock, each tick is 0.1 milliseconds. So the freqeuncy should be multiplied by 10 with 1 subtracted from it
   TIMx->CNT = 0;                       // Ensures counter starts at zero everytime
   TIMx->SR &= ~(1<<0);                 // Resets interupt flag upon counter reset
   while (!(TIMx -> SR & (1<<0)));      // Wait until flag is triggered (Counter == ARR)
   TIMx->SR &= ~(1<<0);                // Clear the interupt flag
}
