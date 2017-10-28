// Information from RM0360 (STM32F030x4/6/8/C and STM32F070x6/B advanced ARM-based 32-bit MCUs), 11.1.3

#pragma once

#include <stddef.h>
#include <assert.h>
#include "../isr-defines.h"
#include "../stm32f030.h"

void USART1_IRQHandler(void);

__attribute__ ((used, section(".isr_vector")))
const IRQHandler_t g_IRQHandlers[] =
{
    // pointer to stack
    (IRQHandler_t)&_estack,

    // core
    Reset_Handler,
    Default_Handler, // NMI_Handler
    Default_Handler, // HardFault_Handler
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    Default_Handler, // SVC_Handler
    0,
    0,
    Default_Handler, // PendSV_Handler
    Default_Handler, // SysTick_Handler,

    // STM32
    Default_Handler, // WWDG_IRQHandler
    0,
    Default_Handler, // RTC_IRQHandler
    Default_Handler, // FLASH_IRQHandler
    Default_Handler, // RCC_IRQHandler
    Default_Handler, // EXTI0_1_IRQHandler
    Default_Handler, // EXTI2_3_IRQHandler
    Default_Handler, // EXTI4_15_IRQHandler
    0,
    Default_Handler, // DMA1_Channel1_IRQHandler
    Default_Handler, // DMA1_Channel2_3_IRQHandler
    Default_Handler, // DMA1_Channel4_5_IRQHandler
    Default_Handler, // ADC_IRQHandler
    Default_Handler, // TIM1_BRK_UP_TRG_COM_IRQHandler
    Default_Handler, // TIM1_CC_IRQHandler
    0,
    Default_Handler, // TIM3_IRQHandler
    Default_Handler, // TIM6_IRQHandler
    0,
    Default_Handler, // TIM14_IRQHandler
    Default_Handler, // TIM15_IRQHandler
    Default_Handler, // TIM16_IRQHandler
    Default_Handler, // TIM17_IRQHandler
    Default_Handler, // I2C1_IRQHandler
    Default_Handler, // I2C2_IRQHandler
    Default_Handler, // SPI1_IRQHandler
    Default_Handler, // SPI2_IRQHandler
    USART1_IRQHandler,
    Default_Handler, // USART2_IRQHandler
    Default_Handler, // USART3_4_5_6_IRQHandler
    0,
    Default_Handler, // USB_IRQHandler
};
static_assert(sizeof(g_IRQHandlers) == (0x00BC + sizeof(g_IRQHandlers[0])), "Wrong ISR vector size");
static_assert(sizeof(g_IRQHandlers)/sizeof(g_IRQHandlers[0]) == (16 + 1 + USB_IRQn), "Wrong ISR vector size");

