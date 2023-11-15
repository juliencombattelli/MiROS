/* Board Support Package (BSP) for the NUCLEO-L152RE board */
#include <stdint.h>  /* Standard integers. WG14/N843 C99 Standard */

#include "bsp.h"
#include "miros.h"
#include "stm32f4xx.h"  /* CMSIS-compliant header file for the MCU used */
/* add other drivers if necessary... */

/* LED pins available on the board (just one user LED LD2--Green on PA.5) */
#define LED_LD2  (1U << 5)

/* Button pins available on the board (just one user Button B1 on PC.13) */
#define BTN_B1   (1U << 13)

#include "circular_buffer.h"

static circular_buf_t history;
static uint8_t hist_buffer[64];

void SysTick_Handler(void) {
    OS_tick();

    __disable_irq();
    OSThread* next = OS_sched();
    circular_buf_put(&history, next->prio);
    __enable_irq();
}

void BSP_init(void) {
    circular_buf_init(&history, hist_buffer, sizeof(hist_buffer));

    /* enable GPIOA clock port for the LED LD2 */
    RCC->AHB1ENR |= (1U << 0);

    /* configure LED (PA.5) pin as push-pull output, no pull-up, pull-down */
    GPIOA->MODER   &= ~((3U << 2*5));
    GPIOA->MODER   |=  ((1U << 2*5));
    GPIOA->OTYPER  &= ~((1U <<   5));
    GPIOA->OSPEEDR &= ~((3U << 2*5));
    GPIOA->OSPEEDR |=  ((1U << 2*5));
    GPIOA->PUPDR   &= ~((3U << 2*5));

    /* enable GPIOC clock port for the Button B1 */
    RCC->AHB1ENR |=  (1U << 2);

    /* configure Button (PC.13) pins as input, no pull-up, pull-down */
    GPIOC->MODER   &= ~(3U << 2*13);
    GPIOC->OSPEEDR &= ~(3U << 2*13);
    GPIOC->OSPEEDR |=  (1U << 2*13);
    GPIOC->PUPDR   &= ~(3U << 2*13);


    /* set pins PA2 (TX) and PA3 (RX) for serial communication */
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN; //enable RCC for port A
    GPIOA->MODER  |= GPIO_MODER_MODER2_1; // PA2 is Alt fn mode (serial TX in this case)
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL2_Pos) ; // That alt fn is alt 7 for PA2
    GPIOA->MODER  |= GPIO_MODER_MODER3_1; // PA3 is Alt fn mode (serial RX in this case)
    GPIOA->AFR[0] |= (7 << GPIO_AFRL_AFSEL3_Pos) ; // Alt fn for PA3 is same as for PA2

    RCC->APB1ENR  |=  RCC_APB1ENR_USART2EN; // enable RCC for USART2

    /* set the baud rate, which requires a mantissa and fraction */
    uint32_t baud_rate = 115200;
    uint16_t uartdiv = SystemCoreClock / baud_rate;
    USART2->BRR = (((uartdiv / 16) << USART_BRR_DIV_Mantissa_Pos)
                 | ((uartdiv % 16) << USART_BRR_DIV_Fraction_Pos));

    /* enable the USART peripheral in both directions */
    USART2->CR1 |= USART_CR1_RE // enable receive
                 | USART_CR1_TE // enable transmit
                 | USART_CR1_UE // enable usart
    ;
}

void BSP_ledRedOn(void) {
    /* no red LED on board */
}

void BSP_ledRedOff(void) {
    /* no red LED on board */
}

void BSP_ledBlueOn(void) {
    /* no blue LED on board */
}

void BSP_ledBlueOff(void) {
    /* no blue LED on board */
}

void BSP_ledGreenOn(void) {
    GPIOA->BSRR |= LED_LD2;  /* turn LED on  */
}

void BSP_ledGreenOff(void) {
    GPIOA->BSRR |= (LED_LD2 << 16);  /* turn LED off */
}

void OS_onStartup(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);

    /* set the SysTick interrupt priority (highest) */
    NVIC_SetPriority(SysTick_IRQn, 0U);
}

void OS_onIdle(void) {
    if (!circular_buf_empty(&history)) {
        char prio_char[2] = {0};
        char c = 0;
        circular_buf_get(&history, &c);
        prio_char[0] = c + '0';
        puts(prio_char);
    }
#ifdef NDBEBUG
    __WFI(); /* stop the CPU and Wait for Interrupt */
#endif
}

void Q_onAssert(char const *module, int loc) {
    /* TBD: damage control */
    (void)module; /* avoid the "unused parameter" compiler warning */
    (void)loc;    /* avoid the "unused parameter" compiler warning */
    NVIC_SystemReset();
}

int putchar(int c)
{
    //if(c=='\r') putchar('\n');
    while( !( USART2->SR & USART_SR_TXE ) ) {}; // wait until we are able to transmit
    USART2->DR = c; // transmit the character
    return c;
}

int puts(const char *s)
{
    while(*s)
        putchar(*s++);
    putchar('\n');
    return 1;
}
