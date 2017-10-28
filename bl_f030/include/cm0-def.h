// Information form the "ARM Cortex-M0 Devices Generic User Guide"

#pragma once

#include <stdint.h>

#define _RW                 volatile           // read-write register
#define _RO                 volatile const     // read-only register
#define _WO                 volatile           // write-only register
#define _RS                                    // unused (reserved) space in registers address range

#define _ALWAYS_INLINE      __attribute__((always_inline))

#define _MASK_TO_POS(mask)  __builtin_ctz(mask)

// == System control block ============================================================================================

struct Scb_t {
	_RO uint32_t CPUID;          // CPUID Base Register
	_RW uint32_t ICSR;           // Interrupt Control and State Register
	_RS uint32_t reserved1;
	_RW uint32_t AIRCR;          // Application Interrupt and Reset Control Register
	_RW uint32_t SCR;            // System Control Register
	_RO uint32_t CCR;            // Configuration and Control Register
	_RS uint32_t reserved2;
	_RW uint32_t SHPR2;          // System Handler Priority Register 2
	_RW uint32_t SHPR3;          // System Handler Priority Register 3
};
static_assert(offsetof(struct Scb_t, SHPR3) == 0x20, "Wrong definition");

#define SCB_AIRCR_VECTKEYSTAT  (0xFFFF << 16)
#define SCB_AIRCR_ENDIANNESS   (   1 << 15)
#define SCB_AIRCR_PRIGROUP     (0x07 << 8)
#define SCB_AIRCR_SYSRESETREQ  (   1 << 2)

#define SCB_SCR_SEVONPEND      (   1 << 4)    // Send Event on Pending
#define SCB_SCR_SLEEPDEEP      (   1 << 2)    // Controls whether the processor uses sleep or deep sleep as its low power mode
#define SCB_SCR_SLEEPONEXIT    (   1 << 1)    // Indicates sleep-on-exit when returning from Handler mode to Thread mode

// == System timer, SysTick ===========================================================================================

struct SysTick_t {
	_RW uint32_t CSR;            // SysTick Control and Status Register
	_RW uint32_t RVR;            // SysTick Reload Value Register
	_RW uint32_t CVR;            // SysTick Current Value Register
	_RO uint32_t CALIB;          // SysTick Calibration Value Register
};
static_assert(offsetof(struct SysTick_t, CALIB) == 0x0C, "Wrong definition");

#define SYSTICK_CSR_COUNTFLAG  (   1 << 16)
#define SYSTICK_CSR_CLKSOURCE  (   1 << 2)
#define SYSTICK_CSR_TICKINT    (   1 << 1)
#define SYSTICK_CSR_ENABLE     (   1 << 0)

// == NVIC ============================================================================================================

struct Nvic_t {
	_RW uint32_t ISER;           // Interrupt Set-Enable Register
	_RS uint32_t reserved1[31];
	_RW uint32_t ICER;           // Interrupt Clear-Enable Register
	_RS uint32_t reserved2[31];
	_RW uint32_t ISPR;           // Interrupt Set-Pending Register
	_RS uint32_t reserved3[31];
	_RW uint32_t ICPR;           // Interrupt Clear-Pending Register
	_RS uint32_t reserved4[95];
	_RW uint8_t  IPR[8];         // Interrupt Priority Register
};
static_assert(offsetof(struct Nvic_t, IPR[0]) == 0x0300, "Wrong definition");

