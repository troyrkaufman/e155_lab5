// main.c
// Troy Kaufman
// tkaufman@hmc.edu
// 10/5/24

#include "main.h"
#define ppr 120

int main(void) {
    // Global direction and speed variables
    int direction;
    int speed;
    int delta_time = 0;

    // Enable PA2 and PA3 as inputs
    gpioEnable(GPIO_PORT_A);
    pinMode(A_PIN, GPIO_INPUT);
    pinMode(B_PIN, GPIO_INPUT);
    //GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD2, 0b01); // Set PA2 as pull-up

    // Initialize timer
    RCC->APB2ENR |= (1<<17); // Timer 16 is for calculating Delta T
    RCC->APB2ENR |= (1<<16); // Timer 15 is for delaying the output to the screen

    // TODO
    // 1. Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= (1 << 0);

    // 2. Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR |= (0b000 << 37);  // PA5   
    SYSCFG->EXTICR |= (0b000 << 41); // PA6    

    // Enable interrupts globally
    __enable_irq();

    //Configure interrupt for rising edge of GPIO pin for button
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
    NVIC->ISER[0] |= (1<<23); // Using EXTI9_5 for pins PA5 and PA6 EXTI Line Interrupts in NVIC. This is set to position 23. 
                              // Then look at corresponding bit position in NVIC_ISERx. Since bit 23 is within the NVIC_ISER0 register
                              // Set bit 23 to 1. 
    
    count_init(TIM16); // Initializes counter
    count_init(TIM15); // Same as delay initialization

    int velocity = direction * speed;

    while (1) {
        delay_update(TIM15, 100);
        printf("The motor's velocity is %d" velocity);
    }
}

// IRQHandler
void EXTI9_5_IRQHandler(void){
    // If A_PIN rising edge is interrupted
    if (EXTI->PR1 & (1 << 5)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 5);
        delta_time = 0;

        // Direction calculation
        direction = B_PIN == 0 ? 1 : -1; // 1 = CW; -1 = CW

        // Update counter for speed calculations
        //while(!(EXTI->PR1 & (1 << 6))){
        //    delta_time = count_update(TIM16, delay_time);
        

        do {
            if (!(EXTI->PR1 & (1 << 6))){            // If B_PIN's rising edge hasn't been detected
                done = 0;                            // Keep counting
            } else {
                done = 1;                            // Stop recounting and return count
            }
            delta_time = count_update(TIM16, done);

        } while (!(EXTI->PR1 & (1 << 6)));

    // if B_PIN rising edge is interrupted
    } else if (EXTI->PR1 & (1 << 6)) {
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 6);

        // Speed calculations
        speed = 1/(ppr * delta_time);

        // Resets delta_time
        delta_time = 0;
    }
}

// On rising edge of A if B is high then CW, on rising A if B is low than CCW for direction

// 120 PPR 