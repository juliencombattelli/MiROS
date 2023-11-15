#include <stdint.h>
#include "miros.h"
#include "bsp.h"

uint32_t stack_blinky1[40];
OSThread blinky1;
void main_blinky1() {
    puts("Starting thread blinky1");
    while (1) {
        BSP_ledGreenOn();
        OS_delay(BSP_TICKS_PER_SEC / 6U);
        BSP_ledGreenOff();
        OS_delay(BSP_TICKS_PER_SEC * 5U / 6U);
    }
}

uint32_t stack_blinky2[40];
OSThread blinky2;
void main_blinky2() {
    puts("Starting thread blinky2");
    while (1) {
        BSP_ledBlueOn();
        OS_delay(BSP_TICKS_PER_SEC * 2U / 8U);
        BSP_ledBlueOff();
        OS_delay(BSP_TICKS_PER_SEC * 6U / 8U);
    }
}

uint32_t stack_blinky3[40];
OSThread blinky3;
void main_blinky3() {
    puts("Starting thread blinky3");
    while (1) {
        BSP_ledRedOn();
        OS_delay(BSP_TICKS_PER_SEC * 4U / 12U);
        BSP_ledRedOff();
        OS_delay(BSP_TICKS_PER_SEC * 8U / 12U);
    }
}

uint32_t stack_idleThread[40];

int main() {
    OS_init(stack_idleThread, sizeof(stack_idleThread));
    BSP_init();

    puts("Starting MiROS");

    /* start blinky1 thread */
    OSThread_start(&blinky1,
                   3U, /* priority */
                   &main_blinky1,
                   stack_blinky1, sizeof(stack_blinky1));

    /* start blinky2 thread */
    OSThread_start(&blinky2,
                   2U, /* priority */
                   &main_blinky2,
                   stack_blinky2, sizeof(stack_blinky2));

    /* start blinky3 thread */
    OSThread_start(&blinky3,
                   1U, /* priority */
                   &main_blinky3,
                   stack_blinky3, sizeof(stack_blinky3));

    /* transfer control to the RTOS to run the threads */
    OS_run();

    return 0;
}
