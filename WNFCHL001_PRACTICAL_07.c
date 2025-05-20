//********************************************************************
//*                    MEC4126F C template                           *
//*==================================================================*
//* WRITTEN BY: Jesse Arendse                                           *
//* DATE CREATED: 07/04/2023                                           *
//* MODIFIED: Dylan Fanner                                            *
//* DATE MODIFIED: 23/01/2025                                         *
//*==================================================================*
//* PROGRAMMED IN: Visual Studio Code                                 *
//* TARGET:        STM32F0                                            *
//*==================================================================*
//********************************************************************
// INCLUDE FILES
//====================================================================

#define STM32F051

//#define USE_LEDS              // Uncomment to use LEDS
#define USE_ADC               // Uncomment to use ADC
#define USE_TIM6              // Uncomment to use TIM6
#define USE_TIM3              // Uncomment to use TIM3
//#define USE_TIM2              // Uncomment to use TIM2

#include "stm32f0xx.h"
#include "stdio.h"
#include "stdint.h"

//====================================================================
// GLOBAL CONSTANTS
//====================================================================
const float P = 700;
const float I = 1;
const float T_s = 0.001;
//====================================================================
// GLOBAL VARIABLES
//====================================================================
volatile float o_p = 0;
volatile float g_p = 0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void set_to_48MHz(void);

#ifdef USE_LEDS
    void init_LEDS(void);
#endif
#ifdef USE_ADC
    void init_ADC(void);
#endif
#ifdef USE_TIM6
    void init_timer6(void);
#endif
#ifdef USE_TIM3
    void init_timer3(void);
#endif
#ifdef USE_TIM2
void init_timer2(void);
#endif

//====================================================================
// MAIN FUNCTION
//====================================================================

int main (void)
{
    set_to_48MHz();

    #ifdef USE_LEDS
        init_LEDS();
    #endif
    #ifdef USE_ADC
        init_ADC();
    #endif
    #ifdef USE_TIM3
        init_timer3();
    #endif
    #ifdef USE_TIM2
        init_timer2();
    #endif
    #ifdef USE_TIM6
        init_timer6();
    #endif

    while (1);
}                           
// End of main

//====================================================================
// ISR DEFINITIONS
//====================================================================
#ifdef USE_TIM2
void TIM2_IRQHandler(void)
{
    volatile uint32_t result = TIM2->CCR1;           // Acknowledge interrupt
    volatile uint32_t seconds = result * 1000;
}
#endif

#ifdef USE_TIM6
void TIM6_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF;            // Acknowledge interrupt

    ADC1->CR |= ADC_CR_ADSTART;                // Start conversion
    while ((ADC1->ISR & ADC_ISR_EOC)==0);
    uint16_t dTheta_c = ADC1->DR;
    while ((ADC1->ISR & ADC_ISR_EOC)==0);
    uint16_t dTheta_f = ADC1->DR;

    // --- UPDATED: Convert raw ADC counts (8-bit) to degrees ---
    float theta_c = ((float)dTheta_c) * (360.0f / 255.0f);
    float theta_f = ((float)dTheta_f) * (360.0f / 255.0f);
    int32_t e = (int32_t)(theta_c - theta_f);
    float   g = e * P;
    // -----------------------------------------------------------

    float o = (g * (2 + I*T_s) + g_p * (I*T_s - 2) + 2 * o_p) / 2;

    if (o > 126)
    {
        o = 126;
    }
    else if (o < -127)
    {
        o = -127;
    }

    TIM3->CCR3 = (int8_t)o + 127;

    g_p = g;
    o_p = o;
}
#endif

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================

void set_to_48MHz(void)
{
    RCC->CFGR |= (0b1010 << 18);                // Set PLL multiplier to 12
    RCC->CFGR |= RCC_CFGR_SW_PLL;               // Set PLL as system clock

    RCC->CR |= RCC_CR_PLLON;                    // Turn on PLL clock
}

#ifdef USE_LEDS
void init_LEDS(void)
{
    // Enable GPIOB clock
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    // Set PB0-7 as output
    GPIOB->MODER |= (GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0)
                    | (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0);
}
#endif

#ifdef USE_TIM2
void init_timer2(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;         // Enable clock to TIM2

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->PUPDR |= 0b01;                       // Write 1 to enable pull up resistor
    GPIOA->MODER |= 0b10;                       // Config MODER0 as AF
    GPIOA->AFR[0] |= 0b10;                      // AFR[0] == AFRL (AF2 = TIM2_CH1)

    TIM2->PSC = 47999;                          // PSC to 1 kHz
    TIM2->ARR = 10000;                          // 10000 counts of 1 ms = 10 s

    // Setup IC
    TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;            // TI1 to IC1
    TIM2->CCMR1 |= TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1; // fcnt, N=8
    TIM2->CCER |= TIM_CCER_CC1E;                // Use CC

    TIM2->DIER |= TIM_DIER_CC1IE;               // Enable CC interrupts

    TIM2->CR1 |= TIM_CR1_CEN;                   // Enable TIM2

    NVIC_EnableIRQ(TIM2_IRQn);
}
#endif

#ifdef USE_TIM3
/*
*   Function to setup output compare on PB0 using TIM3_CH3
*/
void init_timer3(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;         // Enable clock to TIM3

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= 0b10;                       // Config MODER0 as AF
    GPIOB->AFR[0] |= 0b1;                       // AFR[0] == AFRL (AF1 = TIM3_CH3)

    TIM3->PSC = 0;       // no prescaler
    TIM3->ARR = 2399;    // (48 MHz/1)/(2399+1) ≃ 20 kHz PWM

    // Setup PWM
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;
    TIM3->CCMR2 |= TIM_CCMR2_OC3PE;
    TIM3->CCER |= TIM_CCER_CC3E;

    TIM3->CCR3 |= 200;

    TIM3->CR1 |= TIM_CR1_CEN;                   // Enable TIM3
}
#endif

#ifdef USE_TIM6
/*
*   Function to setup 0.001 s delay using TIM6 UE interrupts
*/
void init_timer6(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;         // Enable clock to TIM6
    TIM6->DIER |= TIM_DIER_UIE;                 // Enable update event interrupts

    TIM6->PSC = 119;                            // PSC and ARR for 0.001 s on UE
    TIM6->ARR = 400;

    TIM6->CR1 |= TIM_CR1_ARPE;                  // Use preload registers
    TIM6->CR1 |= TIM_CR1_CEN;                   // Enable TIM6

    NVIC_EnableIRQ(TIM6_IRQn);                  // Allow TIM6 interrupts on NVIC
}
#endif

#ifdef USE_ADC
/*
*   Function to setup ADC conversion on PA6 in cont wait mode
*   with EOC interrupts
*/
void init_ADC(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;          // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER5 | GPIO_MODER_MODER6;          // PA 5 and 6 to Analogue

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;          // Enable ADC clock

    ADC1->CFGR1 |= ADC_CFGR1_RES_1 | ADC_CFGR1_RES_0;  // 8-bit resolution for 1° precision
    ADC1->CFGR1 |= ADC_CFGR1_WAIT;
    ADC1->CHSELR |= ADC_CHSELR_CHSEL5 | ADC_CHSELR_CHSEL6;          // Channel 5 & 6

    //ADC1->IER |= ADC_IER_EOCIE;                 // Enable interrupts on EOC

    ADC1->CR |= ADC_CR_ADEN;                    // Turn on the ADC
    while ((ADC1->ISR & ADC_ISR_ADRDY)==0);     // Wait for ADRDY
}
#endif

//********************************************************************
// END OF PROGRAM
//********************************************************************
