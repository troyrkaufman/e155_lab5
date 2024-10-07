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

int main(void) {

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
    SYSCFG->EXTICR[1] |= (0b000 << 6);  // PA5  (37)           
    SYSCFG->EXTICR[1] |= (0b000 << 10); // PA6   (41) 

    // Enable interrupts globally
    __enable_irq();

    //Configure interrupt for rising edge of GPIO pin for A_PIN and B_PIN
    // 1. Configure mask bit
    EXTI->IMR1 |= (1<<5); // Sets IM0 interrupt
    EXTI->IMR1 |= (1<<6); // Sets IM1 interrupt 
    // 2. Enables rising edge trigger
    EXTI->RTSR1 |= (1<<5); // RT0
    EXTI->RTSR1 |= (1<<6); // RT1
    // 3. Disables falling edge trigger
    EXTI->FTSR1 &= ~(1<<5); // RT0
    EXTI->FTSR1 &= ~(1<<6); // RT1
    // 4. Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1<<EXTI9_5_IRQn); // Using EXTI9_5 for pins PA5 and PA6 EXTI Line Interrupts in NVIC. This is set to position 23. 
                              // Then look at corresponding bit position in NVIC_ISERx. Since bit 23 is within the NVIC_ISER0 register
                              // Set bit 23 to 
    
    count_init(TIM16); // Initializes counter
    delay_init(TIM15); // Initializes delay

    while (1) {
        speed = 1.00/(ppr * delta_time);
        float velocity = direction * speed;
        delay_update(TIM15, 100);
        printf("The motor's velocity is %f", velocity);
    }
}

// IRQHandler
void EXTI9_5_IRQHandler(void){
    // If A_PIN rising edge is interrupted
    int start_count;
    if (EXTI->PR1 & (1 << 5)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 5);
        delta_time = 0;
        start_count = 1;

        // Direction calculation
        direction = B_PIN == 0 ? 1 : -1; // 1 = CW; -1 = CW

        //Start upcounter
        count_update(TIM16, start_count);
                
    // if B_PIN rising edge is interrupted
    } else if (EXTI->PR1 & (1 << 6)) {
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 6);

        // Stop upcounter
        start_count = 0;

        // Update dela_time
        delta_time = count_update(TIM16, start_count);
    }
}