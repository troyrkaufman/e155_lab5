// main.c
// Troy Kaufman
// tkaufman@hmc.edu
// 10/5/24

#include "main.h"
#define ppr 120

// Global variables 
    float direction = 1;
    float speed = 100;
    int delta_time = 0; 
    float velocity;

int main(void) {
    // Buffers for PLL initialization
    configureFlash();
    configureClock();

    // Clock tree management
    RCC -> CFGR |= (0b0000 << RCC_CFGR_HPRE_DIV1); // (4) SYSCLK is by not divided in the AHB PRESC in clock tree
    RCC -> CFGR |= (0b000 << RCC_CFGR_PPRE1_DIV1);  // (8) HCLK (formerly SYSCLK)is not divided by APB1 PRESC (Setting this to 0b0xx avoids 2x multiplier)

    // Enable PA5 (A signal) and PA6 (B signal) as inputs
    gpioEnable(GPIO_PORT_A);
    pinMode(A_PIN, GPIO_INPUT);
    pinMode(B_PIN, GPIO_INPUT);

    // Initialize timers
    RCC->APB2ENR |= (1<<17); // Timer 16 is for calculating Delta T
    RCC->APB2ENR |= (1<<16); // Timer 15 is for delaying the output to the screen

    // 1. Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= (1 << 0);

    // 2. Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[1] |= (0b000 << 4);  // PA5  (37)           
    SYSCFG->EXTICR[1] |= (0b000 << 8); // PA6   (41) 

    // Enable interrupts globally
    __enable_irq();

    // 1. Configure mask bit
    EXTI->IMR1 |= (1<<5); // Sets IM5 interrupt
    EXTI->IMR1 |= (1<<6); // Sets IM6 interrupt 

    // 2. Enables rising edge trigger
    EXTI->RTSR1 |= (1<<5); // RT5
    EXTI->RTSR1 |= (1<<6); // RT6

    // 3. Disables falling edge trigger
    EXTI->FTSR1 &= ~(1<<5); // RT5
    EXTI->FTSR1 &= ~(1<<6); // RT6

    // 4. Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1<<EXTI9_5_IRQn); // Using EXTI9_5 for pins PA5 and PA6 EXTI Line Interrupts in NVIC. This is set to position 23. 
                              // Then look at corresponding bit position in NVIC_ISERx. Since bit 23 is within the NVIC_ISER0 register
                              // Set bit 23 to 

    count_init(TIM16); // Initializes counter for calculating speed
    delay_init(TIM15); // Initializes delay for print statements

    // Counter's input clock
    uint32_t clk_freq = SystemCoreClock / TIM16->PSC;

    // Calculations and print statements
    while (1) {
        if (direction == -1) {
          speed = ((float)clk_freq)/((ppr * (float)delta_time * 4));
        } else if (direction == 1) {
          speed = (((float)clk_freq * 3)/((ppr * (float)delta_time * 4)));
        }
        velocity = direction * speed;
        delay_update(TIM15, 100);
        printf("The motor is spinning at %f rev/s \n", velocity);
       
    }
}

// IRQHandler
void EXTI9_5_IRQHandler(void){
    if (EXTI->PR1 & (1 << 5)){                      // If A pin interrupt is triggered
        EXTI->PR1 |= (1 << 5);                      // If so, clear the interrupt (NB: Write 1 to reset.)
        TIM16->CNT = 0;                             // Ensures the counter starts at zero everytime
        direction = (digitalRead(B_PIN)) ? 1 : -1;  // Direction calculation; 1 = CW; -1 = CCW
    } else if (EXTI->PR1 & (1 << 6)) {              // If B pin interrupt is triggered
        EXTI->PR1 |= (1 << 6);                      // If so, clear the interrupt (NB: Write 1 to reset.)
        delta_time = (int) TIM16->CNT;              // Retrieve the time between interrupts
    }
}