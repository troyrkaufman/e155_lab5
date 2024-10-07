// STM32F401RE_TIM.c
// TIM functions

#include "STM32L432KC_TIM.h"
#include "STM32L432KC_RCC.h"

void count_init(TIM_TypeDef * TIMx){
   TIMx->PSC = 7999;                  // Sets CK_CNT to 10KHz
   TIMx->CR1 |= (1<<7);               // Auto-reload preload enabled
   TIMx->EGR |= (1<<0);               // Initialize all registers to allow preload registers
   //TIMx->ARR = 65534;                   // Sets ARR to maximum value 
   TIMx->CR1 |= (1<<0);               // Start TIMx counter
}

int count_update(TIM_TypeDef * TIMx, uint32_t done){
   int count = 0;
   TIMx->CNT = 0;                       // Ensures counter starts at zero everytime
   while (!done);   // Wait until flag is triggered (Counter == ARR)
   count = TIMx->CNT;
   TIMx->SR &= ~(1<<0);                 // Clear the interrupt flag
   return count;
}

void delay_update(TIM_TypeDef * TIMx, int duration){
   TIMx->ARR = (duration * 10) - 1;            // With a 10KHz clock, each tick is 0.1 milliseconds. So the freqeuncy should be multiplied by 10 with 1 subtracted from it
   TIMx->CNT = 0;                       // Ensures counter starts at zero everytime
   TIMx->SR &= ~(1<<0);                 // Resets interupt flag upon counter reset
   while (!(TIMx -> SR & (1<<0)));      // Wait until flag is triggered (Counter == ARR)
   TIMx->SR &= ~(1<<0);                // Clear the interupt flag
}
