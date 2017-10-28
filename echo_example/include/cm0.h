// Information form the "ARM Cortex-M0 Devices Generic User Guide", rev. r0p0

#pragma once

#include "./cm0-def.h"

// == memory mapping ==================================================================================================

#define SCB             ((struct Scb_t *)      0xE000ED00)
#define SysTick         ((struct SysTick_t *)  0xE000E010)
#define NVIC            ((struct Nvic_t *)     0xE000E100)

// == utility =========================================================================================================

// No Operation
_ALWAYS_INLINE inline void __NOP(void) {
	asm volatile ("nop");
}

// Send Event
_ALWAYS_INLINE inline void __SEV(void) {
	asm volatile ("sev");
}

// Wait For Interrupt
_ALWAYS_INLINE inline void __WFI(void) {
	asm volatile ("wfi");
}

// Wait For Event
_ALWAYS_INLINE inline void __WFE(void) {
	asm volatile ("wfe");
}

// Instruction Synchronization Barrier
_ALWAYS_INLINE inline void __ISB(void) {
	asm volatile ("isb");
}

// Data Synchronization Barrier
_ALWAYS_INLINE inline void __DSB(void) {
	asm volatile ("dsb");
}

// Data Memory Barrier
_ALWAYS_INLINE inline void __DMB(void) {
	asm volatile ("dmb");
}

_ALWAYS_INLINE inline void __enable_irq(void) {
	asm volatile ("cpsie i");
}

_ALWAYS_INLINE inline void __disable_irq(void) {
	asm volatile ("cpsid i");
}

void NVIC_EnableIRQ(uint32_t irq) {
	NVIC->ISER = (1 << irq);
}

void NVIC_DisableIRQ(uint32_t irq) {
	NVIC->ICER = (1 << irq);
}

void NVIC_SystemReset(void) {
	__DSB();
	SCB->AIRCR = (0x5FA << _MASK_TO_POS(SCB_AIRCR_VECTKEYSTAT))  // the key
							 |  SCB_AIRCR_SYSRESETREQ;                       // reset
	__DSB();
	while(1);                                                 // no more return
}

