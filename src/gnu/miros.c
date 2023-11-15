/****************************************************************************
* MInimal Real-time Operating System (MiROS), GNU-ARM port.
* version 1.26 (matching lesson 26, see https://youtu.be/kLxxXNCrY60)
*
* This software is a teaching aid to illustrate the concepts underlying
* a Real-Time Operating System (RTOS). The main goal of the software is
* simplicity and clear presentation of the concepts, but without dealing
* with various corner cases, portability, or error handling. For these
* reasons, the software is generally NOT intended or recommended for use
* in commercial applications.
*
* Copyright (C) 2018 Miro Samek. All Rights Reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
* Git repo:
* https://github.com/QuantumLeaps/MiROS
****************************************************************************/
#include <stdint.h>
#include "miros.h"
#include "qassert.h"

#include "platform.h"

Q_DEFINE_THIS_FILE

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */

OSThread *OS_thread[32 + 1]; /* array of threads started so far */
uint32_t OS_readySet; /* bitmask of threads that are ready to run */
uint32_t OS_delayedSet; /* bitmask of threads that are delayed */

#define LOG2(x) (32U - __builtin_clz(x))

OSThread idleThread;
void main_idleThread() {
    while (1) {
        OS_onIdle();
    }
}

static inline void OS_triggerContextSwitch(void) {
    // Set the PendSV exception as pending to perform the context switch
    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

void OS_init(void *stkSto, uint32_t stkSize) {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    NVIC_SetPriority(PendSV_IRQn, 0xFF);

    /* start idleThread thread */
    OSThread_start(&idleThread,
                   0U, /* idle thread priority */
                   &main_idleThread,
                   stkSto, stkSize);
}

OSThread* OS_sched(void) {
    /* choose the next thread to execute... */
    OSThread *next;
    if (OS_readySet == 0U) { /* idle condition? */
        next = OS_thread[0]; /* the idle thread */
    }
    else {
        next = OS_thread[LOG2(OS_readySet)];
        Q_ASSERT(next != (OSThread *)0);
    }

    /* trigger PendSV, if needed */
    if (next != OS_curr) {
        OS_next = next;
        OS_triggerContextSwitch();
    }

    return next;
}

void OS_run(void) {
    /* callback to configure and start interrupts */
    OS_onStartup();

    __disable_irq();
    OS_sched();
    __enable_irq();

    /* the following code should never execute */
    Q_ERROR();
}

void OS_tick(void) {
    uint32_t workingSet = OS_delayedSet;
    while (workingSet != 0U) {
        OSThread *t = OS_thread[LOG2(workingSet)];
        uint32_t bit;
        Q_ASSERT((t != (OSThread *)0) && (t->timeout != 0U));

        bit = (1U << (t->prio - 1U));
        --t->timeout;
        if (t->timeout == 0U) {
            OS_readySet   |= bit;  /* insert to set */
            OS_delayedSet &= ~bit; /* remove from set */
        }
        workingSet &= ~bit; /* remove from working set */
    }
}

void OS_delay(uint32_t ticks) {
    uint32_t bit;
    __disable_irq();

    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_curr != OS_thread[0]);

    OS_curr->timeout = ticks;
    bit = (1U << (OS_curr->prio - 1U));
    OS_readySet &= ~bit;
    OS_delayedSet |= bit;
    OS_sched();
    __enable_irq();
}

static uintptr_t OS_pushStackFrame(uint32_t* sp, uintptr_t pc) {
    *(--sp) = (1U << 24); /* xPSR */
    *(--sp) = pc;         /* PC */
    *(--sp) = 0;          /* LR  */
    *(--sp) = 0x12121212; /* R12 */
    *(--sp) = 0x03030303; /* R3  */
    *(--sp) = 0x02020202; /* R2  */
    *(--sp) = 0x01010101; /* R1  */
    *(--sp) = 0x00000000; /* R0  */ /* use it to store a user void pointer */
    /* push additional non-saved registers according to AAPCS */
    *(--sp) = 0x11111111; /* R11 */
    *(--sp) = 0x10101010; /* R10 */
    *(--sp) = 0x09090909; /* R9 */
    *(--sp) = 0x08080808; /* R8 */
    *(--sp) = 0x07070707; /* R7 */
    *(--sp) = 0x06060606; /* R6 */
    *(--sp) = 0x05050505; /* R5 */
    *(--sp) = 0x04040404; /* R4 */

    return sp;
}

void OSThread_start(
    OSThread *me,
    uint8_t prio, /* thread priority */
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    /* round up the bottom of the stack to the 8-byte boundary */
    uint32_t *stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* priority must be in range and the priority level must be unused */
    Q_REQUIRE((prio < Q_DIM(OS_thread)) && (OS_thread[prio] == (OSThread *)0));

    sp = OS_pushStackFrame(sp, threadHandler);

    /* save the top of the stack in the thread's attribute */
    me->sp = sp;

    /* pre-fill the unused part of the stack */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEF;
    }

    /* register the thread with the OS */
    OS_thread[prio] = me;
    me->prio = prio;
    /* make the thread ready to run */
    if (prio > 0U) {
        OS_readySet |= (1U << (prio - 1U));
    }
}

/**
 *
 * @brief PendSV exception handler, handling context switches
 *
 * The PendSV exception is the only execution context in the system that can
 * perform context switching. When an execution context finds out it has to
 * switch contexts, it pends the PendSV exception.
 *
 * When PendSV is pended, the decision that a context switch must happen has
 * already been taken. In other words, when PendSV_Handler() runs, we *know* we
 * have to swap *something*.
 * 
 * Implementation based on 'Use of SVCall and PendSV to avoid critical code
 * regions' on page B1-641 of the ARMv7-M architecture reference manual.
 */
void PendSV_Handler(void) {
    // Push callee-save registers
    asm volatile ("push {r4-r11}");
    // Save current stack pointer into the suspended task context
    OS_curr->sp = __get_MSP();
    // Set the stack pointer to the next executed task
    __set_MSP(OS_next->sp);
    // Replace the current task pointer with the next executed one
    OS_curr = OS_next;
    // Pop the callee-save registers
    asm volatile ("pop {r4-r11}");
}
