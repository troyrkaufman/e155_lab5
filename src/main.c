// main.c
// Troy Kaufman
// tkaufman@hmc.edu
// 10/5/24

#include "main.h"
#define ppr 120
#define N 20
#define sysclk 100000

// Global variables 
int direction = 1;
volatile uint32_t delta_time = 0; 
volatile uint32_t prev_cnt = 0;
volatile uint32_t cnt_value;
// Store the previous states of A_PIN and B_PIN globally
volatile uint8_t prev_A_state = 0;
volatile uint8_t prev_B_state = 0;

int main(void) {
    // Buffers for PLL initialization
    configureFlash();
    configureClock();

    // Clock tree management
    RCC -> CFGR |= (0b0000 << RCC_CFGR_HPRE_DIV1); // (4) SYSCLK is by not divided in the AHB PRESC in clock tree
    RCC -> CFGR |= (0b000 << RCC_CFGR_PPRE1_DIV1); // (8) HCLK (formerly SYSCLK)is not divided by APB1 PRESC (Setting this to 0b0xx avoids 2x multiplier)

    // Enable PA5 (A signal) and PA6 (B signal) as inputs
    gpioEnable(GPIO_PORT_A);
    pinMode(A_PIN, GPIO_INPUT);
    pinMode(B_PIN, GPIO_INPUT);

    // Initialize timers
    RCC->APB2ENR |= (1<<17);            // Timer 16 is for calculating Delta T
    RCC->APB2ENR |= (1<<16);            // Timer 15 is for delaying the output to the screen

    // 1. Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= (1 << 0);

    // 2. Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[1] |= (0b000 << 4);  // PA5             
    SYSCFG->EXTICR[1] |= (0b000 << 8);  // PA6   

    // Enable interrupts globally
    __enable_irq();

    // 1. Configure mask bit
    EXTI->IMR1 |= (1<<5);               // Sets IM8 interrupt
    EXTI->IMR1 |= (1<<6);               // Sets IM6 interrupt 

    // 2. Enables rising edge trigger
    EXTI->RTSR1 |= (1<<5);              // RT5
    EXTI->RTSR1 |= (1<<6);              // RT6

    // 3. Enables falling edge trigger
    EXTI->FTSR1 |= (1<<5);              // RT5
    EXTI->FTSR1 |= (1<<6);              // RT6

    // 4. Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1<<EXTI9_5_IRQn); // Using EXTI9_5 for pins PA8 and PA6 EXTI Line Interrupts in NVIC. This is set to position 23. 
                                        // Then look at corresponding bit position in NVIC_ISERx. Since bit 23 is within the NVIC_ISER0 register
                                        // Set bit 23 to 

    count_init(TIM16);                  // Initializes counter for calculating speed
    delay_init(TIM15);                  // Initializes delay for print statements

    // Variables for averaging
    float speed = 0;
    float velocity = 0;
    float velocity_readings[N];         // Array to store recent velocity readings
    int index = 0;                      // Index to keep track of where to insert the new reading
    float sum = 0;                      // Sum of the last N velocity readings
    float avg_velocity = 0;             // Variable to store the average velocity
    int count = 0;                      // Counter to track how many readings have been stored

    while (1) {
        
        // Calculate current velocity
        if (delta_time != 0) {
            speed = (sysclk)/((ppr * (float)delta_time * 4));  // Calculate the speed
            velocity = direction * speed;                      // Calculate the velocity
        } else {
            velocity = 0;
        }

        // Calculate rolling average of velocities
        if (count == N) {
            sum -= velocity_readings[index];  // Throw out the oldest velocity in the list
        }

        velocity_readings[index] = velocity;  // Store the new velocity reading
        sum += velocity;                      // Add the new velocity to the sum
        index = (index + 1) % N;              // Increment the index and wrap around if necessary

        if (count < N) {                      // If the buffer isn't full yet, increase the count
            count++;
        }

        if (count == N) {                     // Only calculate average once we have enough readings (after N samples)
            avg_velocity = sum / N;
        } else {
            avg_velocity = sum / count;       // Use the sum so far for the initial average
        }

        // Delay then print average velocity to window
        delay_update(TIM15, 1000);            
        printf("The motor is spinning at %f rev/s \n", avg_velocity); 
}
}

void EXTI9_5_IRQHandler(void){
    // Read the current count in TIM16 counter
    cnt_value = TIM16->CNT;

    // Read the current states of A_PIN and B_PIN once at the beginning of the interrupt
    uint8_t current_A_state = digitalRead(A_PIN);
    uint8_t current_B_state = digitalRead(B_PIN);

    /////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////// If A pin interrupt is triggered ////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    if (EXTI->PR1 & (1 << 5)){                         
      EXTI->PR1 |= (1 << 5);                           // If so, clear the interrupt (NB: Write 1 to reset.)
      
      // Retrieve the time between interrupts
      if (cnt_value >= prev_cnt) {
        delta_time = cnt_value - prev_cnt;             // Normal case: No overflow
      } else {
        delta_time = (0xffff - prev_cnt + cnt_value);  // Handle overflow case for 16-bit timer
      }

      // Determine direction based on A_PIN state change
      if (current_A_state != prev_A_state) {                         // Only update if A_PIN state has changed
          direction = (current_B_state != current_A_state) ? 1 : -1; // 1 = CW, -1 = CCW
      }

      // Update previous state and count
      prev_A_state = current_A_state;
      prev_cnt = cnt_value;                           

    /////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////// If B pin interrupt is triggered ////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////
    } else if (EXTI->PR1 & (1 << 6)) {              
      EXTI->PR1 |= (1 << 6);                         // If so, clear the interrupt (NB: Write 1 to reset.)

      if (cnt_value >= prev_cnt) {
        delta_time = cnt_value - prev_cnt;           // Normal case: No overflow
      } else {
        delta_time = (0xffff - prev_cnt + cnt_value);                // Handle overflow case for 16-bit timer
      }

      // Determine direction based on A_PIN state change
      if (current_A_state != prev_A_state) {                         // Only update if A_PIN state has changed
          direction = (current_A_state != current_B_state) ? 1 : -1; // 1 = CW, -1 = CCW
      }

      // Update previous state and count
      prev_A_state = current_A_state;
      prev_cnt = cnt_value;                        
    }
}