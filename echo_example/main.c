#include "stm32f030_f4.h"
#include "startup-common.h"
#include "isr/stm32f030.h"

#define CPU_F               8000000
#define BAUD                 115200

void USART1_IRQHandler(void) {
    if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
        uint8_t c = USART1->RDR;
        USART1->TDR = c;
        GPIOA->ODR ^= (1 << 4);  // toggle PA4
    }
}

int main(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_IOPAEN;

    GPIOA->MODER |= (1 << (4*2));    // PA4 to generic output

    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_UP | GPIO_PUPDR_PUPDR10_UP;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9|GPIO_MODER_MODER10))
        | (GPIO_MODER_MODER9_ALT | GPIO_MODER_MODER10_ALT);
    GPIOA->AFRH = (GPIOA->AFRH &~ (GPIO_AFRH_AFR9 | GPIO_AFRH_AFR10))
        | (1 << (1 * 4)) | (1 << (2 * 4));

    USART1->BRR = CPU_F / BAUD;
    USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

    NVIC_EnableIRQ(USART1_IRQn);

    while(1) {}

    return 0;
}

