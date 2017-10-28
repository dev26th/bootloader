#include <stdint.h>

#define RCC_AHBENR    *((volatile uint32_t *)0x40021014)
#define GPIOA_MODER   *((volatile uint32_t *)0x48000000)
#define GPIOA_ODR     *((volatile uint32_t *)0x48000014)

static void delay(uint32_t n) {
    n *= 1000;
    for(volatile uint32_t i = 0; i < n; ++i);
}

static void Reset_Handler(void) {
    RCC_AHBENR |= (1 << 17);        // enable GPIOA clock
    GPIOA_MODER |= (1 << (4*2));    // PA4 to generic output

    while(1) {
        for(uint8_t i = 0; i < 3; ++i) {
            GPIOA_ODR &= ~(1 << 4);
            delay(100);
            GPIOA_ODR |= (1 << 4);
            delay(100);
        }

        delay(400);
    }
}

__attribute__ ((used, section(".isr_vector")))
void (* const g_pfnVectors[])(void) = {
    (void (*)(void))0x20001000,
    Reset_Handler
};

