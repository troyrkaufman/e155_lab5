// main.c
// Troy Kaufman
// tkaufman@hmc.edu
// 10/5/24

#include "main.h"
#define ppr 120

// Global direction and speed variables
    float direction = 1;
    float speed = 100;
    int delta_time = 0;
    volatile int start_count;                          // cnt variable 

int main(void) {
    RCC -> CFGR |= (0b0000 << RCC_CFGR_HPRE_DIV1); // (4) SYSCLK is not divided in the AHB PRESC in clock tree 
    RCC -> CFGR |= (0b000 << RCC_CFGR_PPRE1_DIV1);  // (8) HCLK (formerly SYSCLK)is not divided by APB1 PRESC (Setting this to 0b0xx avoids 2x multiplier)

    // Enable PA2 and PA3 as inputs
    gpioEnable(GPIO_PORT_A);
    pinMode(A_PIN, GPIO_INPUT);
    pinMode(B_PIN, GPIO_INPUT);
    //GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b01); // Set PA2 as pull-up

    // Initialize timers
    RCC->APB2ENR |= (1<<17); // Timer 16 is for calculating Delta T
    RCC->APB2ENR |= (1<<16); // Timer 15 is for delaying the output to the screen

    // 1. Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= (1 << 0);

    // 2. Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[1] |= (0b000 << 4);  // PA5  (37)           
    //SYSCFG->EXTICR[1] |= (0b000 << 8); // PA6   (41) 

    // Enable interrupts globally
    __enable_irq();

    //Configure interrupt for rising edge of GPIO pin for A_PIN and B_PIN
    // 1. Configure mask bit
    EXTI->IMR1 |= (1<<5); // Sets IM0 interrupt
    //EXTI->IMR1 |= (1<<6); // Sets IM1 interrupt 

    // 2. Enables rising edge trigger
    EXTI->RTSR1 |= (1<<5); // RT0
    //EXTI->RTSR1 |= (1<<6); // RT1

    // 3. Disables falling edge trigger
    EXTI->FTSR1 &= ~(1<<5); // RT0
    //EXTI->FTSR1 &= ~(1<<6); // RT1

    // 4. Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1<<EXTI9_5_IRQn); // Using EXTI9_5 for pins PA5 and PA6 EXTI Line Interrupts in NVIC. This is set to position 23. 
                              // Then look at corresponding bit position in NVIC_ISERx. Since bit 23 is within the NVIC_ISER0 register
                              // Set bit 23 to 


    // B_PIN
    SYSCFG->EXTICR[1] |= (0b000 << 8); // PA6   (41) 
    EXTI->IMR1 |= (1<<6);             // Sets IM1 interrupt 
    EXTI->RTSR1 |= (1<<6);            // Enables rising edge trigger
    EXTI->FTSR1 &= ~(1<<6);           // Disables falling edge trigger
    NVIC->ISER[0] |= (1<<EXTI9_5_IRQn);
    
    count_init(TIM16); // Initializes counter
    delay_init(TIM15); // Initializes delay

    while (1) {
        speed = 1.00/((ppr * (float)delta_time * 1e-6));
        float velocity = direction * speed;
        //delay_update(TIM15, 100);
        for (int i = 0; i <100000; i++);
        if (delta_time != 0){
           printf("The motor's velocity is %f\n", velocity);
        }
       
    }
}

// IRQHandler
void EXTI9_5_IRQHandler(void){
    

    if (EXTI->PR1 & (1 << 5)){                // If A_PIN rising edge is interrupted

        EXTI->PR1 |= (1 << 5);                // If so, clear the interrupt (NB: Write 1 to reset.)

        //delta_time = 0;                       // Reset delta_time

        //delta_time = (int) TIM16->CNT;

        TIM16->CNT = 0;                    // Ensures counter starts at zero everytime

        start_count = 1;                      // Start upcounter 

        direction = (digitalRead(B_PIN)) ? 1 : -1;      // Direction calculation; 1 = CW; -1 = CCW

        //count_update(TIM16);     // Start counting time between interrupts
                
    } else if (EXTI->PR1 & (1 << 6)) {        // if B_PIN's interrupt is triggered

        EXTI->PR1 |= (1 << 6);                // If so, clear the interrupt (NB: Write 1 to reset.)

        start_count = 0;                      // Stop upcounter

        delta_time = (int) TIM16->CNT;

        TIM16->CNT = 0;                    // Ensures counter starts at zero everytime

       // delta_time = count_update(TIM16);  // Update delta_time
    }
}