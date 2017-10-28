// Information from STM32F103xC, STM32F103xD, STM32F103xE Datasheet

#pragma once

#include "stm32f030.h"

// == Hardware ========================================================================================================

#define HAS_TIM3_HW
#define HAS_TIM6_HW
#define HAS_TIM7_HW
#define HAS_TIM14_HW
#define HAS_RTC_HW
#define HAS_WWDG_HW
#define HAS_IWDG_HW
#define HAS_SPI2_HW
#define HAS_USART2_HW
#define HAS_USART3_HW
#define HAS_USART4_HW
#define HAS_USART5_HW
#define HAS_I2C1_HW
#define HAS_I2C2_HW
#define HAS_SYSCFG_HW
#define HAS_EXTI_HW
#define HAS_USART6_HW
#define HAS_ADC_HW
#define HAS_TIM1_HW
#define HAS_SPI1_HW
#define HAS_USART1_HW
#define HAS_TIM15_HW
#define HAS_TIM16_HW
#define HAS_TIM17_HW
#define HAS_DBGMCU_HW
#define HAS_DMA_HW
#define HAS_RCC_HW
#define HAS_FLASH_HW
#define HAS_CRC_HW
#define HAS_GPIOA_HW
#define HAS_GPIOB_HW
#define HAS_GPIOC_HW
#define HAS_GPIOD_HW
#define HAS_GPIOF_HW

// == Memory Mapping ==================================================================================================

#define TIM3            ((struct TIM_t *)      0x40000400)
#define TIM6            ((struct TIM_t *)      0x40001000)
#define TIM7            ((struct TIM_t *)      0x40001400)
#define TIM14           ((struct TIM_t *)      0x40002000)
#define RTC             ((struct RTC_t *)      0x40002800)
#define WWDG            ((struct WWDG_t *)     0x40002C00)
#define IWDG            ((struct IWDG_t *)     0x40003000)
#define SPI2            ((struct SPI_t *)      0x40003800)
#define USART2          ((struct USART_t *)    0x40004400)
#define USART3          ((struct USART_t *)    0x40004800)
#define USART4          ((struct USART_t *)    0x40004C00)
#define USART5          ((struct USART_t *)    0x40005000)
#define I2C1            ((struct I2C_t *)      0x40005400)
#define I2C2            ((struct I2C_t *)      0x40005800)
#define PWR             ((struct PWR_t *)      0x40007000)
#define SYSCFG          ((struct SYSCFG_t *)   0x40010000)
#define EXTI            ((struct EXTI_t *)     0x40010400)
#define USART6          ((struct USART_t *)    0x40011400)
#define ADC             ((struct ADC_t *)      0x40012400)
#define TIM1            ((struct TIM_t *)      0x40012C00)
#define SPI1            ((struct SPI_t *)      0x40013000)
#define USART1          ((struct USART_t *)    0x40013800)
#define TIM15           ((struct TIM_t *)      0x40014000)
#define TIM16           ((struct TIM_t *)      0x40014400)
#define TIM17           ((struct TIM_t *)      0x40014800)
#define DBGMCU          ((struct DBGMCU_t *)   0x40015800)
#define DMA             ((struct DMA_t *)      0x40020000)
#define RCC             ((struct RCC_t *)      0x40021000)
#define FLASH           ((struct FLASH_t *)    0x40022000)
#define CRC             ((struct CRC_t *)      0x40023000)
#define GPIOA           ((struct GPIO_t *)     0x48000000)
#define GPIOB           ((struct GPIO_t *)     0x48000400)
#define GPIOC           ((struct GPIO_t *)     0x48000800)
#define GPIOD           ((struct GPIO_t *)     0x48000C00)
#define GPIOF           ((struct GPIO_t *)     0x48001400)

