// main.c
// Troy Kaufman
// tkaufman@hmc.edu
// 10/5/24

#include "main.h"
#define ppr 120
#define N 60

// Global variables 
    float direction = 1;
    float speed = 100;
    volatile uint32_t delta_time = 0; 
    volatile uint32_t prev_cnt = 0;
    float velocity;
    volatile uint32_t cnt_value;

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
    SYSCFG->EXTICR[2] |= (0b000 << 0);  // PA8             
    SYSCFG->EXTICR[1] |= (0b000 << 8); // PA6   (41) 

    // Enable interrupts globally
    __enable_irq();

    // 1. Configure mask bit
    EXTI->IMR1 |= (1<<8); // Sets IM5 interrupt
    EXTI->IMR1 |= (1<<6); // Sets IM6 interrupt 

    // 2. Enables rising edge trigger
    EXTI->RTSR1 |= (1<<8); // RT8
    EXTI->RTSR1 |= (1<<6); // RT6

    // 3. Enables falling edge trigger
    EXTI->FTSR1 |= (1<<8); // RT8
    EXTI->FTSR1 |= (1<<6); // RT6

    // 4. Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1<<EXTI9_5_IRQn); // Using EXTI9_5 for pins PA8 and PA6 EXTI Line Interrupts in NVIC. This is set to position 23. 
                              // Then look at corresponding bit position in NVIC_ISERx. Since bit 23 is within the NVIC_ISER0 register
                              // Set bit 23 to 

    count_init(TIM16); // Initializes counter for calculating speed
    delay_init(TIM15); // Initializes delay for print statements

    // Counter's input clock
    uint32_t clk_freq = SystemCoreClock / (TIM16->PSC - 1);
    float sum_velo = 0;
    float avg_velo = 0;
    int index = 0;
    float velocity_readings[N] = {0};
    int initialized = 0;

    // Calculations and print statements
    while (1) {
    
       float velocity_readings[N];  // Array to store recent velocity readings
int index = 0;  // Index to keep track of where to insert the new reading
float sum = 0;  // Sum of the last N velocity readings
float avg_velocity = 0;  // Variable to store the average velocity
int count = 0;  // Counter to track how many readings have been stored

while (1) {
    if (delta_time != 0) {
        // Calculate the speed and velocity
        speed = ((float)clk_freq)/((ppr * (float)delta_time * 2));
        velocity = direction * speed;
    } else {
        velocity = 0;
    }

    // Subtract the oldest reading from the sum (if buffer is full)
    if (count == N) {
        sum -= velocity_readings[index];
    }

    // Store the new velocity reading
    velocity_readings[index] = velocity;

    // Add the new velocity to the sum
    sum += velocity;

    // Increment the index and wrap around if necessary
    index = (index + 1) % N;

    // If the buffer isn't full yet, increase the count
    if (count < N) {
        count++;
    }

    // Only calculate average once we have enough readings (after N samples)
    if (count == N) {
        avg_velocity = sum / N;
    } else {
        // Use the sum so far for the initial average
        avg_velocity = sum / count;
    }

    // Delay for readability and testing
    delay_update(TIM15, 1000);

    // Print the averaged velocity
    printf("The motor is spinning at %f rev/s \n", avg_velocity);
}
}
}

void EXTI9_5_IRQHandler(void){
    cnt_value = TIM16->CNT;
    if (EXTI->PR1 & (1 << 5)){                      // If A pin interrupt is triggered
          EXTI->PR1 |= (1 << 5);                    // If so, clear the interrupt (NB: Write 1 to reset.)

          if (cnt_value >= prev_cnt) {
            delta_time = cnt_value - prev_cnt;     // Normal case: No overflow
          } else {
            delta_time = (0xffff - prev_cnt + cnt_value);  // Handle overflow case for 16-bit timer
          }

            prev_cnt = cnt_value;                    // Keep track of count

            // Only update direction when there is a change in A_PIN
        if (digitalRead(A_PIN) == 1) {
            if (digitalRead(B_PIN) == 0) {
                direction = 1; // Clockwise
            } else {
                direction = -1; // Counter-clockwise
            }
        } else {
            if (digitalRead(B_PIN) == 1) {
                direction = 1; // Clockwise
            } else {
                direction = -1; // Counter-clockwise
            }
        }
          

    } else if (EXTI->PR1 & (1 << 6)) {              // If B pin interrupt is triggered
          EXTI->PR1 |= (1 << 6);                    // If so, clear the interrupt (NB: Write 1 to reset.)

          if (cnt_value >= prev_cnt) {
            delta_time = cnt_value - prev_cnt;     // Normal case: No overflow
          } else {
            delta_time = (0xffff - prev_cnt + cnt_value);  // Handle overflow case for 16-bit timer
          }

          prev_cnt = cnt_value;                    // Keep track of count

          // Only update direction when there is a change in A_PIN
        if (digitalRead(A_PIN) == 1) {
            if (digitalRead(B_PIN) == 0) {
                direction = 1; // Clockwise
            } else {
                direction = -1; // Counter-clockwise
            }
        } else {
            if (digitalRead(B_PIN) == 1) {
                direction = 1; // Clockwise
            } else {
                direction = -1; // Counter-clockwise
            }
        }
    }
}

















/*
// IRQHandler
void EXTI9_5_IRQHandler(void){
    if (EXTI->PR1 & (1 << 5)){                      // If A pin interrupt is triggered
        EXTI->PR1 |= (1 << 5);                      // If so, clear the interrupt (NB: Write 1 to reset.)

        if (digitalRead(A_PIN) == 1) {              // A_PIN rising edge
          prev_cnt = TIM16->CNT;                    // Keep track of count
          delta_time = TIM16->CNT - prev_cnt; 
          TIM16->CNT = 0;                           // Reset counter to avoid overflow
          direction = (digitalRead(B_PIN)) ? 1 : -1;// Direction calculation; 1 = CW; -1 = CCW
        } else {                                    // A_PIN falling edge
          prev_cnt = TIM16->CNT;
          delta_time = TIM16->CNT - prev_cnt;       // Retrieve time between interrupts
          direction = (digitalRead(B_PIN)) ? 1 : -1;// Direction calculation; 1 = CW; -1 = CCW
          TIM16->CNT = 0;                           // Rest counter to avoid overflow
        }

    } else if (EXTI->PR1 & (1 << 6)) {              // If B pin interrupt is triggered
        EXTI->PR1 |= (1 << 6);                      // If so, clear the interrupt (NB: Write 1 to reset.)

         if (digitalRead(B_PIN) == 1) {             // B_PIN rising edge
          prev_cnt = TIM16->CNT;                    // Keep track of current count in TIM16
          delta_time = TIM16->CNT;                  // Retrieve time between interrutps
          direction = (digitalRead(B_PIN)) ? 1 : -1;// Direction calculation; 1 = CW; -1 = CCW
        } else {                                    // B_PIN falling edge
          prev_cnt = TIM16->CNT;
          delta_time = TIM16->CNT - prev_cnt;
          direction = (digitalRead(B_PIN)) ? 1 : -1;// Direction calculation; 1 = CW; -1 = CCW

        }
    }
} */