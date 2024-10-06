// STM32F401RE_TIM.c
// TIM functions

#include "STM32L432KC_TIM.h"
#include "STM32L432KC_RCC.h"

#define CK_CNT 10000

void pwm_init(TIM_TypeDef * TIMx){
    TIMx->PSC = 99;                      // Setting prescaler to 99 to get CK_CNT = 10k Hz from 1 MHz clock input
    TIMx->CCMR1 |= (0b110<<4);          // Setting to OUPUT PWM mode TIMx_CNT < TIMx_CCR1 else inactive
    TIMx->CCMR1 |= (1<<3);              // Preload register on TIMxCCR1 is enabled
    TIMx->BDTR |= (1<<15);              // (MOE) OC and OCN outputs are enabled if their respective enable bits are set (CCxE, CCxNE, in TIMx_CCER register)
    TIMx->CCER |= (1<<0);               // Configure Channel 1 as output
    TIMx->CR1 |= (1<<7);                // Auto-reload preload enabled. The ARR is now buffered
    TIMx->CCER &= ~(1<<1);               // Configure output as active HIGH
    TIMx->EGR |= (1<<0);                // Initialize all registers to allow preload registers to transfer to shadow registers during an update event
    TIMx->CR1 |= (1<<0);                // Enables clock input after timer configuration ***LAST STEP*** 
}

void pwm_update(TIM_TypeDef * TIMx, uint32_t freq){
    int arr;
    if (freq == 0){
      TIMx->CR1 &= ~(1<<0);
    } else {
      TIMx->CR1 |= (1<<0);
      arr = (CK_CNT / freq) - 1;       // Calculation for ARR
      TIMx->ARR = arr;                 // Sets PWM frequency to requested amount
      TIMx->CCR1 = arr/2;              // Sets duty cycle to 50%
      TIMx->EGR &= ~(1<<0);            // Resets the flag
    }
}

void delay_init(TIM_TypeDef * TIMx){
   TIMx->PSC = 99;                    //(CK_CNT / 1000) - 1;//(CK_CNT / 1000) - 1; // Creating one ms resolution from system clock
   TIMx->CR1 |= (1<<7);               // Auto-reload preload enabled
   TIMx->EGR |= (1<<0);               // Initialize all registers to allow preload registers
   TIMx->CR1 |= (1<<0);               // Start tIM15 counter
}

void delay_update(TIM_TypeDef * TIMx, uint32_t duration){
   TIMx->ARR = (duration * 10) - 1;     // With a 10KHz clock, each tick is 0.1 milliseconds. So the freqeuncy should be multiplied by 10 with 1 subtracted from it
   TIMx->CNT = 0;                       // Ensures counter starts at zero everytime
   TIMx->SR &= ~(1<<0);                 // Resets interupt flag upon counter reset
   while (!(TIMx -> SR & (1<<0)));      // Wait until flag is triggered (Counter == ARR)
   TIMx->SR &= ~(1<<0);                 // Clear the interupt flag
}

/*
void initTIM(TIM_TypeDef * TIMx){
  // Set prescaler to give 1 ms time base
  uint32_t psc_div = (uint32_t) ((SystemCoreClock/1e3));

  // Set prescaler division factor
  TIMx->PSC = (psc_div - 1);
  // Generate an update event to update prescaler value
  TIMx->EGR |= 1;
  // Enable counter
  TIMx->CR1 |= 1; // Set CEN = 1
}

void delay_millis(TIM_TypeDef * TIMx, uint32_t ms){
  TIMx->ARR = ms;// Set timer max count
  TIMx->EGR |= 1;     // Force update
  TIMx->SR &= ~(0x1); // Clear UIF
  TIMx->CNT = 0;      // Reset count

  while(!(TIMx->SR & 1)); // Wait for UIF to go high
}
*/