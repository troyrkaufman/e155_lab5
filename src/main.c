// main.c
// Troy Kaufman
// tkaufman@hmc.edu
// 10/5/24

#include "main.h"

int main(void) {
    // Global direction and speed variables
    int direction;
    int speed;

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

    // TODO: Configure interrupt for rising edge of GPIO pin for button
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

    printf()
}

// TODO: What is the right name for the IRQHandler?
void XXXXXX(void){
    // Check that the button was what triggered our interrupt
    if (EXTI->PR1 & (1 << )){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << );

        // Then toggle the LED
        togglePin(LED_PIN);



    // Put speed calculation here inside the handler

    }
}

