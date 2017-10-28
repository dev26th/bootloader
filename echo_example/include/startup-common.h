#pragma once

#include <stdint.h>
#include <assert.h>
#include "./isr-defines.h"
#include "std-dummies.h"

// Reaction for unexpected interrupt
void SystemError(void) {
    while(1);
}

void __assert_func(const char * file, int line, const char * func, const char * cond) {
    (void)file; (void)line; (void)func; (void)cond;
    while(1);
}

extern int main(void);

// addresses defined by the linker script
extern uint32_t _estack;   // stack (actually end of RAM)
extern uint32_t _sidata;   // begin of data in ROM, which will be copied to RAM
extern uint32_t _sdata;    // start of data in RAM
extern uint32_t _edata;    // end of data in RAM
extern uint32_t _sbss;     // start of BSS in RAM - will be cleared
extern uint32_t _ebss;     // end of BSS in RAM

void Reset_Handler(void) {
    // copy data
    for(uint32_t *t = &_sdata, *f = &_sidata; t < &_edata; ++t, ++f) *t = *f;

    // clear BSS
    for(uint32_t *t = &_sbss; t < &_ebss; ++t) *t = 0;

    main(); // should not return
}

// is used for all undefined interrupt routines
void Default_Handler(void) {
    SystemError();
}

