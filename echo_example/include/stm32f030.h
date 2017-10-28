// Information from RM0360 (STM32F030x4/6/8/C and STM32F070x6/B advanced ARM-based 32-bit MCUs)

#pragma once

#include <stddef.h>
#include <assert.h>
#include "cm0.h"

// == Base addresses and values =======================================================================================

#define HSI_VALUE            8000000    // HSI speed in Hz
#define HSI14_VALUE          14000000   // HSI14 (ADCCLK) speed in Hz
#define FLASH_BASE           0x08000000 // flash base address
#define SRAM_BASE            0x20000000 // RAM base address
#define PERIPH_BASE          0x40000000 // periphery base address

// == Interrupts ======================================================================================================

// Core
#define SVC_IRQn                  ((uint32_t)-5)
#define PendSV_IRQn               ((uint32_t)-2)
#define SysTick_IRQn              ((uint32_t)-1)

// STM32
#define WWDG_IRQn                 0
#define RTC_IRQn                  2
#define FLASH_IRQn                3
#define RCC_IRQn                  4
#define EXTI0_1_IRQn              5
#define EXTI2_3_IRQn              6
#define EXTI4_15_IRQn             7
#define DMA1_Channel1_IRQn        9
#define DMA1_Channel2_3_IRQn      10
#define DMA1_Channel4_5_IRQn      11
#define ADC_IRQn                  12
#define TIM1_BRK_UP_TRG_COM_IRQn  13
#define TIM1_CC_IRQn              14
#define TIM3_IRQn                 16
#define TIM6_IRQn                 17
#define TIM14_IRQn                19
#define TIM15_IRQn                20
#define TIM16_IRQn                21
#define TIM17_IRQn                22
#define I2C1_IRQn                 23
#define I2C2_IRQn                 24
#define SPI1_IRQn                 25
#define SPI2_IRQn                 26
#define USART1_IRQn               27
#define USART2_IRQn               28
#define USART3_4_5_6_IRQn         29
#define USB_IRQn                  31

// == CRC =============================================================================================================

struct CRC_t {
	_RW uint32_t DR;             // Data register
	_RW uint8_t  IDR;            // Independent data register
	_RS uint8_t  reserved1;
	_RS uint8_t  reserved2;
	_RS uint8_t  reserved3;
	_RW uint32_t CR;             // Control register
	_RS uint32_t reserved4;
	_RW uint32_t INIT;           // Initial CRC value
};
static_assert(offsetof(struct CRC_t, INIT) == 0x10, "Wrong definition");

#define CRC_DR_DATA                        (0xFFFFFFFF << 0) // Data register bits

#define CRC_IDR_DATA                       (0xFF << 0)       // General-purpose 8-bit data register bits

#define CRC_CR_REVOUT                      (   1 << 7)       // Reverse output data
#define CRC_CR_REVIN                       (0x03 << 5)       // Reverse input data
#define CRC_CR_REVIN_NONE                  (0x00 << 5)       //   - Bit order not affected
#define CRC_CR_REVIN_BYTE                  (0x01 << 5)       //   - Bit reversal done by byte
#define CRC_CR_REVIN_HWORD                 (0x02 << 5)       //   - Bit reversal done by half-word
#define CRC_CR_REVIN_WORD                  (0x03 << 5)       //   - Bit reversal done by word
#define CRC_CR_RESET                       (   1 << 0)       // RESET bit

#define CRC_INIT_INIT                      (0xFFFFFFFF << 0) // Programmable initial CRC value

// == PWR =============================================================================================================

struct PWR_t {
	_RW uint32_t CR;             // Control register
	_RW uint32_t CSR;            // Control/status register
};
static_assert(offsetof(struct PWR_t, CSR) == 0x04, "Wrong definition");

#define PWR_CR_DBP                         (   1 << 8)       // Disable RTC domain write protection
#define PWR_CR_CSBF                        (   1 << 3)       // Clear standby flag
#define PWR_CR_CWUF                        (   1 << 2)       // Clear wakeup flag
#define PWR_CR_PDDS                        (   1 << 1)       // Power down deep sleep
#define PWR_CR_LPDS                        (   1 << 0)       // Low-power deep sleep

#define PWR_CSR_EWUP6                      (   1 << 14)      // Enable WKUP6 pin
#define PWR_CSR_EWUP5                      (   1 << 13)      // Enable WKUP5 pin
#define PWR_CSR_EWUP4                      (   1 << 12)      // Enable WKUP4 pin
#define PWR_CSR_EWUP3                      (   1 << 11)      // Enable WKUP3 pin
#define PWR_CSR_EWUP2                      (   1 << 9)       // Enable WKUP2 pin
#define PWR_CSR_EWUP1                      (   1 << 8)       // Enable WKUP1 pin
#define PWR_CSR_SBF                        (   1 << 1)       // Standby flag
#define PWR_CSR_WUF                        (   1 << 0)       // Wakeup flag

// == RCC =============================================================================================================

struct RCC_t {
	_RW uint32_t CR;             // Clock control register
	_RW uint32_t CFGR;           // Clock configuration register
	_RW uint32_t CIR;            // Clock interrupt register
	_RW uint32_t APB2RSTR;       // APB2 peripheral reset register
	_RW uint32_t APB1RSTR;       // APB1 peripheral reset register
	_RW uint32_t AHBENR;         // AHB peripheral clock enable register
	_RW uint32_t APB2ENR;        // APB2 peripheral clock enable register
	_RW uint32_t APB1ENR;        // APB1 peripheral clock enable register
	_RW uint32_t BDCR;           // RTC domain control register
	_RW uint32_t CSR;            // Control/status register
	_RW uint32_t AHBRSTR;        // AHB peripheral reset register
	_RW uint32_t CFGR2;          // Clock configuration register 2
	_RW uint32_t CFGR3;          // Clock configuration register 3
	_RW uint32_t CR2;            // Clock control register 2
};
static_assert(offsetof(struct RCC_t, CR2) == 0x34, "Wrong definition");

#define RCC_CR_PLLRDY                      (   1 << 25)      // PLL clock ready flag
#define RCC_CR_PLLON                       (   1 << 24)      // PLL enable
#define RCC_CR_CSSON                       (   1 << 19)      // Clock security system enable
#define RCC_CR_HSEBYP                      (   1 << 18)      // External high-speed clock bypass
#define RCC_CR_HSERDY                      (   1 << 17)      // External high-speed clock ready flag
#define RCC_CR_HSEON                       (   1 << 16)      // External high-speed clock enable
#define RCC_CR_HSICAL                      (0xFF << 8)       // Internal high-speed clock calibration
#define RCC_CR_HSITRIM                     (0x1F << 3)       // Internal high-speed clock trimming
#define RCC_CR_HSIRDY                      (   1 << 1)       // Internal high-speed clock ready flag
#define RCC_CR_HSION                       (   1 << 0)       // Internal high-speed clock enable

#define RCC_CFGR_PLLNODIV                  (   1 << 31)      // PLL clock not divided for MCO (not available on STM32F030x8 devices)
#define RCC_CFGR_MCOPRE                    (0x07 << 28)      // Microcontroller Clock Output Prescaler (not available on STM32F030x8 devices)
#define RCC_CFGR_MCO                       (0x0F << 24)      // MCU clock output
#define RCC_CFGR_MCO_NOCLOCK               (0x00 << 24)      //   - MCO output disabled, no clock on MCO
#define RCC_CFGR_MCO_HSI14                 (0x01 << 24)      //   - Internal RC 14 MHz (HSI14) oscillator clock selected
#define RCC_CFGR_MCO_LSI                   (0x02 << 24)      //   - Internal low speed (LSI) oscillator clock selected
#define RCC_CFGR_MCO_LSE                   (0x03 << 24)      //   - External low speed (LSE) oscillator clock selected
#define RCC_CFGR_MCO_SYSCLK                (0x04 << 24)      //   - System clock selected
#define RCC_CFGR_MCO_HSI                   (0x05 << 24)      //   - Internal RC 8 MHz (HSI) oscillator clock selected
#define RCC_CFGR_MCO_HSE                   (0x06 << 24)      //   - External 4-32 MHz (HSE) oscillator clock selected
#define RCC_CFGR_MCO_PLL                   (0x07 << 24)      //   - PLL clock selected (divided by 1 or 2, depending on PLLNODIV)
#define RCC_CFGR_PLLMUL                    (0x0F << 18)      // PLL multiplication factor
#define RCC_CFGR_PLLXTPRE                  (   1 << 17)      // HSE divider for PLL input clock (same as PREDIV[0] from RCC_CFGR2)
#define RCC_CFGR_PLLXTPRE_HSE              (   0 << 17)
#define RCC_CFGR_PLLXTPRE_HSEDIV2          (   1 << 17)
#define RCC_CFGR_PLLSRC                    (   1 << 16)      // PLL entry clock source
#define RCC_CFGR_PLLSRC_HSIDIV2            (   0 << 16)      //   - HSI oscillator clock / 2 selected as PLL input clock
#define RCC_CFGR_PLLSRC_HSE                (   1 << 16)      //   - HSE/PREDIV selected as PLL input clock
#define RCC_CFGR_ADCPRE                    (0x03 << 14)      // ADC prescaler
#define RCC_CFGR_PPRE                      (0x07 << 8)       // PCLK prescaler (APB)
#define RCC_CFGR_HPRE                      (0x0F << 4)       // AHB prescaler
#define RCC_CFGR_SWS                       (0x03 << 2)       // System clock switch status
#define RCC_CFGR_SWS_HSI                   (0x00 << 2)       //   - HSI oscillator used as system clock
#define RCC_CFGR_SWS_HSE                   (0x01 << 2)       //   - HSE oscillator used as system clock
#define RCC_CFGR_SWS_PLL                   (0x02 << 2)       //   - PLL used as system clock
#define RCC_CFGR_SW                        (0x03 << 0)       // System clock switch
#define RCC_CFGR_SW_HSI                    (0x00 << 0)       //   - HSI selected as system clock
#define RCC_CFGR_SW_HSE                    (0x01 << 0)       //   - HSE selected as system clock
#define RCC_CFGR_SW_PLL                    (0x02 << 0)       //   - PLL selected as system clock

#define RCC_CIR_CSSC                       (   1 << 23)      // Clock security system interrupt clear
#define RCC_CIR_HSI14RDYC                  (   1 << 21)      // HSI14 ready interrupt clear
#define RCC_CIR_PLLRDYC                    (   1 << 20)      // PLL ready interrupt clear
#define RCC_CIR_HSERDYC                    (   1 << 19)      // HSE ready interrupt clear
#define RCC_CIR_HSIRDYC                    (   1 << 18)      // HSI ready interrupt clear
#define RCC_CIR_LSERDYC                    (   1 << 17)      // LSE ready interrupt clear
#define RCC_CIR_LSIRDYC                    (   1 << 16)      // LSI ready interrupt clear
#define RCC_CIR_HSI14RDYIE                 (   1 << 13)      // HSI14 ready interrupt enable
#define RCC_CIR_PLLRDYIE                   (   1 << 12)      // PLL ready interrupt enable
#define RCC_CIR_HSERDYIE                   (   1 << 11)      // HSE ready interrupt enable
#define RCC_CIR_HSIRDYIE                   (   1 << 10)      // HSI ready interrupt enable
#define RCC_CIR_LSERDYIE                   (   1 << 9)       // LSE ready interrupt enable
#define RCC_CIR_LSIRDYIE                   (   1 << 8)       // LSI ready interrupt enable
#define RCC_CIR_CSSF                       (   1 << 7)       // Clock security system interrupt flag
#define RCC_CIR_HSI14RDYF                  (   1 << 5)       // HSI14 ready interrupt flag
#define RCC_CIR_PLLRDYF                    (   1 << 4)       // PLL ready interrupt flag
#define RCC_CIR_HSERDYF                    (   1 << 3)       // HSE ready interrupt flag
#define RCC_CIR_HSIRDYF                    (   1 << 2)       // HSI ready interrupt flag
#define RCC_CIR_LSERDYF                    (   1 << 1)       // LSE ready interrupt flag
#define RCC_CIR_LSIRDYF                    (   1 << 0)       // LSI ready interrupt flag

#define RCC_APB2RSTR_DBGMCURST             (   1 << 22)      // Debug MCU reset
#define RCC_APB2RSTR_TIM17RST              (   1 << 18)      // TIM17 reset
#define RCC_APB2RSTR_TIM16RST              (   1 << 17)      // TIM16 reset
#define RCC_APB2RSTR_TIM15RST              (   1 << 16)      // TIM15 reset
#define RCC_APB2RSTR_USART1RST             (   1 << 14)      // USART1 reset
#define RCC_APB2RSTR_SPI1RST               (   1 << 12)      // SPI1 reset
#define RCC_APB2RSTR_TIM1RST               (   1 << 11)      // TIM1 reset
#define RCC_APB2RSTR_ADCRST                (   1 << 9)       // ADC reset
#define RCC_APB2RSTR_USART6RST             (   1 << 5)       // USART6 reset
#define RCC_APB2RSTR_SYSCFGRST             (   1 << 0)       // SYSCFG reset

#define RCC_APB1RSTR_PWRRST                (   1 << 28)      // Power interface reset
#define RCC_APB1RSTR_USBRST                (   1 << 23)      // USB reset
#define RCC_APB1RSTR_I2C2RST               (   1 << 22)      // I2C2 reset
#define RCC_APB1RSTR_I2C1RST               (   1 << 21)      // I2C1 reset
#define RCC_APB1RSTR_USART5RST             (   1 << 20)      // USART5 reset
#define RCC_APB1RSTR_USART4RST             (   1 << 19)      // USART4 reset
#define RCC_APB1RSTR_USART3RST             (   1 << 18)      // USART3 reset
#define RCC_APB1RSTR_USART2RST             (   1 << 17)      // USART2 reset
#define RCC_APB1RSTR_SPI2RST               (   1 << 14)      // SPI2 reset
#define RCC_APB1RSTR_WWDGRST               (   1 << 11)      // Window watchdog reset
#define RCC_APB1RSTR_TIM14RST              (   1 << 8)       // TIM14 reset
#define RCC_APB1RSTR_TIM7RST               (   1 << 5)       // TIM7 reset
#define RCC_APB1RSTR_TIM6RST               (   1 << 4)       // TIM6 reset
#define RCC_APB1RSTR_TIM3RST               (   1 << 1)       // TIM3 reset

#define RCC_AHBENR_IOPFEN                  (   1 << 22)      // I/O port F clock enable
#define RCC_AHBENR_IOPDEN                  (   1 << 20)      // I/O port D clock enable
#define RCC_AHBENR_IOPCEN                  (   1 << 19)      // I/O port C clock enable
#define RCC_AHBENR_IOPBEN                  (   1 << 18)      // I/O port B clock enable
#define RCC_AHBENR_IOPAEN                  (   1 << 17)      // I/O port A clock enable
#define RCC_AHBENR_CRCEN                   (   1 << 6)       // CRC clock enable
#define RCC_AHBENR_FLITFEN                 (   1 << 4)       // FLITF clock enable
#define RCC_AHBENR_SRAMEN                  (   1 << 2)       // SRAM interface clock enable
#define RCC_AHBENR_DMAEN                   (   1 << 0)       // DMA clock enable

#define RCC_APB2ENR_DBGMCUEN               (   1 << 22)      // MCU debug module clock enable
#define RCC_APB2ENR_TIM17EN                (   1 << 18)      // TIM17 clock enable
#define RCC_APB2ENR_TIM16EN                (   1 << 17)      // TIM16 clock enable
#define RCC_APB2ENR_TIM15EN                (   1 << 16)      // TIM15 clock enable
#define RCC_APB2ENR_USART1EN               (   1 << 14)      // USART1 clock enable
#define RCC_APB2ENR_SPI1EN                 (   1 << 12)      // SPI1 clock enable
#define RCC_APB2ENR_TIM1EN                 (   1 << 11)      // TIM1 clock enable
#define RCC_APB2ENR_ADCEN                  (   1 << 9)       // ADC clock enable
#define RCC_APB2ENR_USART6EN               (   1 << 5)       // USART6 clock enable
#define RCC_APB2ENR_SYSCFGEN               (   1 << 0)       // SYSCFG clock enable

#define RCC_APB1ENR_PWREN                  (   1 << 28)      // Power interface clock enable
#define RCC_APB1ENR_USBEN                  (   1 << 23)      // USB clock enable
#define RCC_APB1ENR_I2C2EN                 (   1 << 22)      // I2C2 clock enable
#define RCC_APB1ENR_I2C1EN                 (   1 << 21)      // I2C1 clock enable
#define RCC_APB1ENR_USART5EN               (   1 << 20)      // USART5 clock enable
#define RCC_APB1ENR_USART4EN               (   1 << 19)      // USART4 clock enable
#define RCC_APB1ENR_USART3EN               (   1 << 18)      // USART3 clock enable
#define RCC_APB1ENR_USART2EN               (   1 << 17)      // USART2 clock enable
#define RCC_APB1ENR_SPI2EN                 (   1 << 14)      // SPI2 clock enable
#define RCC_APB1ENR_WWDGEN                 (   1 << 11)      // Window watchdog clock enable
#define RCC_APB1ENR_TIM14EN                (   1 << 8)       // TIM14 clock enable
#define RCC_APB1ENR_TIM7EN                 (   1 << 5)       // TIM7 clock enable
#define RCC_APB1ENR_TIM6EN                 (   1 << 4)       // TIM6 clock enable
#define RCC_APB1ENR_TIM3EN                 (   1 << 1)       // TIM3 clock enable

#define RCC_BDCR_BDRST                     (   1 << 16)      // RTC domain software reset
#define RCC_BDCR_RTCEN                     (   1 << 15)      // RTC clock enable
#define RCC_BDCR_RTCSEL                    (0x03 << 8)       // RTC clock source selection
#define RCC_BDCR_RTCSEL_NOCLOCK            (0x00 << 8)       //   - No clock
#define RCC_BDCR_RTCSEL_LSE                (0x01 << 8)       //   - LSE oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_LSI                (0x02 << 8)       //   - LSI oscillator clock used as RTC clock
#define RCC_BDCR_RTCSEL_HSE                (0x03 << 8)       //   - HSE oscillator clock divided by 32 used as RTC clock
#define RCC_BDCR_LSEDRV                    (0x03 << 3)       // LSE oscillator drive capability
#define RCC_BDCR_LSEDRV_LOWER              (0x00 << 8)       //   - lower driving capability
#define RCC_BDCR_LSEDRV_LOW                (0x01 << 8)       //   - medium low driving capability
#define RCC_BDCR_LSEDRV_HIGH               (0x02 << 8)       //   - medium high driving capability
#define RCC_BDCR_LSEDRV_HIGHER             (0x03 << 8)       //   - higher driving capability (reset value)
#define RCC_BDCR_LSEBYP                    (   1 << 2)       // External low-speed oscillator bypass
#define RCC_BDCR_LSERDY                    (   1 << 1)       // External low-speed oscillator ready
#define RCC_BDCR_LSEON                     (   1 << 0)       // External low-speed oscillator enable

#define RCC_CSR_LPWRRSTF                   (   1 << 31)      // Low-power reset flag
#define RCC_CSR_WWDGRSTF                   (   1 << 30)      // Window watchdog reset flag
#define RCC_CSR_IWDGRSTF                   (   1 << 29)      // Independent watchdog reset flag
#define RCC_CSR_SFTRSTF                    (   1 << 28)      // Software reset flag
#define RCC_CSR_PORRSTF                    (   1 << 27)      // POR/PDR reset flag
#define RCC_CSR_PINRSTF                    (   1 << 26)      // PIN reset flag
#define RCC_CSR_OBLRSTF                    (   1 << 25)      // Option byte loader reset flag
#define RCC_CSR_RMVF                       (   1 << 24)      // Remove reset flag
#define RCC_CSR_V18PWRRSTF                 (   1 << 23)      // Reset flag of the 1.8 V domain
#define RCC_CSR_LSIRDY                     (   1 << 1)       // Internal low-speed oscillator ready
#define RCC_CSR_LSION                      (   1 << 0)       // Internal low-speed oscillator enable

#define RCC_AHBRSTR_IOPFRST                (   1 << 22)      // I/O port F reset
#define RCC_AHBRSTR_IOPDRST                (   1 << 20)      // I/O port D reset
#define RCC_AHBRSTR_IOPCRST                (   1 << 19)      // I/O port C reset
#define RCC_AHBRSTR_IOPBRST                (   1 << 18)      // I/O port B reset
#define RCC_AHBRSTR_IOPARST                (   1 << 17)      // I/O port A reset

#define RCC_CFGR2_PREDIV                   (0x0F << 0)       // PREDIV division factor

#define RCC_CFGR3_ADCSW                    (   1 << 8)       // ADC clock source selection
#define RCC_CFGR3_USBSW                    (   1 << 7)       // USB clock source selection
#define RCC_CFGR3_I2C1SW                   (   1 << 4)       // I2C1 clock source selection
#define RCC_CFGR3_USART1SW                 (0x03 << 0)       // USART1 clock source selection
#define RCC_CFGR3_USART1SW_PCLK            (0x00 << 0)       //   - PCLK selected as USART1 clock source (default)
#define RCC_CFGR3_USART1SW_SYSCLK          (0x01 << 0)       //   - System clock (SYSCLK) selected as USART1 clock
#define RCC_CFGR3_USART1SW_LSE             (0x02 << 0)       //   - LSE clock selected as USART1 clock
#define RCC_CFGR3_USART1SW_HSI             (0x03 << 0)       //   - HSI clock selected as USART1 clock

#define RCC_CR2_HSI14CAL                   (0xFF << 8)       // HSI14 clock calibration
#define RCC_CR2_HSI14TRIM                  (0x1F << 3)       // HSI14 clock trimming
#define RCC_CR2_HSI14DIS                   (   1 << 2)       // HSI14 clock request from ADC disable
#define RCC_CR2_HSI14RDY                   (   1 << 1)       // HSI14 clock ready flag
#define RCC_CR2_HSI14ON                    (   1 << 0)       // HSI14 clock enable

// == GPIO ============================================================================================================

struct GPIO_t {
	_RW uint32_t MODER;          // Port mode register
	_RW uint32_t OTYPER;         // Port output type register
	_RW uint32_t OSPEEDR;        // Port output speed register
	_RW uint32_t PUPDR;          // Port pull-up/pull-down register
	_RO uint32_t IDR;            // Port input data register
	_RW uint32_t ODR;            // Port output data register
	_WO uint32_t BSRR;           // Port bit set/reset register
	_RW uint32_t LCKR;           // Port configuration lock register
	_RW uint32_t AFRL;           // Alternate function low register
	_RW uint32_t AFRH;           // Alternate function high register
	_WO uint32_t BRR;            // Port bit reset register
};
static_assert(offsetof(struct GPIO_t, BRR) == 0x28, "Wrong definition");

#define GPIO_MODER_MODER15                 (0x03 << 30)      // Pin 15 mode bits
#define GPIO_MODER_MODER15_INPUT           (0x00 << 30)      //   - Input mode (reset state)
#define GPIO_MODER_MODER15_OUTPUT          (0x01 << 30)      //   - General purpose output mode
#define GPIO_MODER_MODER15_ALT             (0x02 << 30)      //   - Alternate function mode
#define GPIO_MODER_MODER15_ANALOG          (0x03 << 30)      //   - Analog mode
#define GPIO_MODER_MODER14                 (0x03 << 28)      // Pin 14 mode bits
#define GPIO_MODER_MODER14_INPUT           (0x00 << 28)      //   - Input mode (reset state)
#define GPIO_MODER_MODER14_OUTPUT          (0x01 << 28)      //   - General purpose output mode
#define GPIO_MODER_MODER14_ALT             (0x02 << 28)      //   - Alternate function mode
#define GPIO_MODER_MODER14_ANALOG          (0x03 << 28)      //   - Analog mode
#define GPIO_MODER_MODER13                 (0x03 << 26)      // Pin 13 mode bits
#define GPIO_MODER_MODER13_INPUT           (0x00 << 26)      //   - Input mode (reset state)
#define GPIO_MODER_MODER13_OUTPUT          (0x01 << 26)      //   - General purpose output mode
#define GPIO_MODER_MODER13_ALT             (0x02 << 26)      //   - Alternate function mode
#define GPIO_MODER_MODER13_ANALOG          (0x03 << 26)      //   - Analog mode
#define GPIO_MODER_MODER12                 (0x03 << 24)      // Pin 12 mode bits
#define GPIO_MODER_MODER12_INPUT           (0x00 << 24)      //   - Input mode (reset state)
#define GPIO_MODER_MODER12_OUTPUT          (0x01 << 24)      //   - General purpose output mode
#define GPIO_MODER_MODER12_ALT             (0x02 << 24)      //   - Alternate function mode
#define GPIO_MODER_MODER12_ANALOG          (0x03 << 24)      //   - Analog mode
#define GPIO_MODER_MODER11                 (0x03 << 22)      // Pin 11 mode bits
#define GPIO_MODER_MODER11_INPUT           (0x00 << 22)      //   - Input mode (reset state)
#define GPIO_MODER_MODER11_OUTPUT          (0x01 << 22)      //   - General purpose output mode
#define GPIO_MODER_MODER11_ALT             (0x02 << 22)      //   - Alternate function mode
#define GPIO_MODER_MODER11_ANALOG          (0x03 << 22)      //   - Analog mode
#define GPIO_MODER_MODER10                 (0x03 << 20)      // Pin 10 mode bits
#define GPIO_MODER_MODER10_INPUT           (0x00 << 20)      //   - Input mode (reset state)
#define GPIO_MODER_MODER10_OUTPUT          (0x01 << 20)      //   - General purpose output mode
#define GPIO_MODER_MODER10_ALT             (0x02 << 20)      //   - Alternate function mode
#define GPIO_MODER_MODER10_ANALOG          (0x03 << 20)      //   - Analog mode
#define GPIO_MODER_MODER9                  (0x03 << 18)      // Pin 9 mode bits
#define GPIO_MODER_MODER9_INPUT            (0x00 << 18)      //   - Input mode (reset state)
#define GPIO_MODER_MODER9_OUTPUT           (0x01 << 18)      //   - General purpose output mode
#define GPIO_MODER_MODER9_ALT              (0x02 << 18)      //   - Alternate function mode
#define GPIO_MODER_MODER9_ANALOG           (0x03 << 18)      //   - Analog mode
#define GPIO_MODER_MODER8                  (0x03 << 16)      // Pin 8 mode bits
#define GPIO_MODER_MODER8_INPUT            (0x00 << 16)      //   - Input mode (reset state)
#define GPIO_MODER_MODER8_OUTPUT           (0x01 << 16)      //   - General purpose output mode
#define GPIO_MODER_MODER8_ALT              (0x02 << 16)      //   - Alternate function mode
#define GPIO_MODER_MODER8_ANALOG           (0x03 << 16)      //   - Analog mode
#define GPIO_MODER_MODER7                  (0x03 << 14)      // Pin 7 mode bits
#define GPIO_MODER_MODER7_INPUT            (0x00 << 14)      //   - Input mode (reset state)
#define GPIO_MODER_MODER7_OUTPUT           (0x01 << 14)      //   - General purpose output mode
#define GPIO_MODER_MODER7_ALT              (0x02 << 14)      //   - Alternate function mode
#define GPIO_MODER_MODER7_ANALOG           (0x03 << 14)      //   - Analog mode
#define GPIO_MODER_MODER6                  (0x03 << 12)      // Pin 6 mode bits
#define GPIO_MODER_MODER6_INPUT            (0x00 << 12)      //   - Input mode (reset state)
#define GPIO_MODER_MODER6_OUTPUT           (0x01 << 12)      //   - General purpose output mode
#define GPIO_MODER_MODER6_ALT              (0x02 << 12)      //   - Alternate function mode
#define GPIO_MODER_MODER6_ANALOG           (0x03 << 12)      //   - Analog mode
#define GPIO_MODER_MODER5                  (0x03 << 10)      // Pin 5 mode bits
#define GPIO_MODER_MODER5_INPUT            (0x00 << 10)      //   - Input mode (reset state)
#define GPIO_MODER_MODER5_OUTPUT           (0x01 << 10)      //   - General purpose output mode
#define GPIO_MODER_MODER5_ALT              (0x02 << 10)      //   - Alternate function mode
#define GPIO_MODER_MODER5_ANALOG           (0x03 << 10)      //   - Analog mode
#define GPIO_MODER_MODER4                  (0x03 << 8)       // Pin 4 mode bits
#define GPIO_MODER_MODER4_INPUT            (0x00 << 8)       //   - Input mode (reset state)
#define GPIO_MODER_MODER4_OUTPUT           (0x01 << 8)       //   - General purpose output mode
#define GPIO_MODER_MODER4_ALT              (0x02 << 8)       //   - Alternate function mode
#define GPIO_MODER_MODER4_ANALOG           (0x03 << 8)       //   - Analog mode
#define GPIO_MODER_MODER3                  (0x03 << 6)       // Pin 3 mode bits
#define GPIO_MODER_MODER3_INPUT            (0x00 << 6)       //   - Input mode (reset state)
#define GPIO_MODER_MODER3_OUTPUT           (0x01 << 6)       //   - General purpose output mode
#define GPIO_MODER_MODER3_ALT              (0x02 << 6)       //   - Alternate function mode
#define GPIO_MODER_MODER3_ANALOG           (0x03 << 6)       //   - Analog mode
#define GPIO_MODER_MODER2                  (0x03 << 4)       // Pin 2 mode bits
#define GPIO_MODER_MODER2_INPUT            (0x00 << 4)       //   - Input mode (reset state)
#define GPIO_MODER_MODER2_OUTPUT           (0x01 << 4)       //   - General purpose output mode
#define GPIO_MODER_MODER2_ALT              (0x02 << 4)       //   - Alternate function mode
#define GPIO_MODER_MODER2_ANALOG           (0x03 << 4)       //   - Analog mode
#define GPIO_MODER_MODER1                  (0x03 << 2)       // Pin 1 mode bits
#define GPIO_MODER_MODER1_INPUT            (0x00 << 2)       //   - Input mode (reset state)
#define GPIO_MODER_MODER1_OUTPUT           (0x01 << 2)       //   - General purpose output mode
#define GPIO_MODER_MODER1_ALT              (0x02 << 2)       //   - Alternate function mode
#define GPIO_MODER_MODER1_ANALOG           (0x03 << 2)       //   - Analog mode
#define GPIO_MODER_MODER0                  (0x03 << 0)       // Pin 0 mode bits
#define GPIO_MODER_MODER0_INPUT            (0x00 << 0)       //   - Input mode (reset state)
#define GPIO_MODER_MODER0_OUTPUT           (0x01 << 0)       //   - General purpose output mode
#define GPIO_MODER_MODER0_ALT              (0x02 << 0)       //   - Alternate function mode
#define GPIO_MODER_MODER0_ANALOG           (0x03 << 0)       //   - Analog mode

#define GPIO_OTYPER_OT15                   (   1 << 15)      // Pin 15 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT14                   (   1 << 14)      // Pin 14 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT13                   (   1 << 13)      // Pin 13 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT12                   (   1 << 12)      // Pin 12 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT11                   (   1 << 11)      // Pin 11 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT10                   (   1 << 10)      // Pin 10 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT9                    (   1 << 9)       // Pin 9 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT8                    (   1 << 8)       // Pin 8 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT7                    (   1 << 7)       // Pin 7 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT6                    (   1 << 6)       // Pin 6 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT5                    (   1 << 5)       // Pin 5 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT4                    (   1 << 4)       // Pin 4 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT3                    (   1 << 3)       // Pin 3 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT2                    (   1 << 2)       // Pin 2 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT1                    (   1 << 1)       // Pin 1 configuration bits (0 - push-pull, 1 - open-drain)
#define GPIO_OTYPER_OT0                    (   1 << 0)       // Pin 0 configuration bits (0 - push-pull, 1 - open-drain)

#define GPIO_OSPEEDR_OSPEEDR15             (0x03 << 30)      // Pin 15 output speed
#define GPIO_OSPEEDR_OSPEEDR15_LOW         (0x00 << 30)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR15_MEDIUM      (0x01 << 30)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR15_HIGH        (0x03 << 30)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR14             (0x03 << 28)      // Pin 14 output speed
#define GPIO_OSPEEDR_OSPEEDR14_LOW         (0x00 << 28)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR14_MEDIUM      (0x01 << 28)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR14_HIGH        (0x03 << 28)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR13             (0x03 << 26)      // Pin 13 output speed
#define GPIO_OSPEEDR_OSPEEDR13_LOW         (0x00 << 26)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR13_MEDIUM      (0x01 << 26)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR13_HIGH        (0x03 << 26)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR12             (0x03 << 24)      // Pin 12 output speed
#define GPIO_OSPEEDR_OSPEEDR12_LOW         (0x00 << 24)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR12_MEDIUM      (0x01 << 24)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR12_HIGH        (0x03 << 24)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR11             (0x03 << 22)      // Pin 11 output speed
#define GPIO_OSPEEDR_OSPEEDR11_LOW         (0x00 << 22)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR11_MEDIUM      (0x01 << 22)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR11_HIGH        (0x03 << 22)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR10             (0x03 << 20)      // Pin 10 output speed
#define GPIO_OSPEEDR_OSPEEDR10_LOW         (0x00 << 20)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR10_MEDIUM      (0x01 << 20)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR10_HIGH        (0x03 << 20)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR9              (0x03 << 18)      // Pin 9 output speed
#define GPIO_OSPEEDR_OSPEEDR9_LOW          (0x00 << 18)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR9_MEDIUM       (0x01 << 18)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR9_HIGH         (0x03 << 18)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR8              (0x03 << 16)      // Pin 8 output speed
#define GPIO_OSPEEDR_OSPEEDR8_LOW          (0x00 << 16)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR8_MEDIUM       (0x01 << 16)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR8_HIGH         (0x03 << 16)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR7              (0x03 << 14)      // Pin 7 output speed
#define GPIO_OSPEEDR_OSPEEDR7_LOW          (0x00 << 14)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR7_MEDIUM       (0x01 << 14)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR7_HIGH         (0x03 << 14)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR6              (0x03 << 12)      // Pin 6 output speed
#define GPIO_OSPEEDR_OSPEEDR6_LOW          (0x00 << 12)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR6_MEDIUM       (0x01 << 12)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR6_HIGH         (0x03 << 12)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR5              (0x03 << 10)      // Pin 5 output speed
#define GPIO_OSPEEDR_OSPEEDR5_LOW          (0x00 << 10)      //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR5_MEDIUM       (0x01 << 10)      //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR5_HIGH         (0x03 << 10)      //   - High speed
#define GPIO_OSPEEDR_OSPEEDR4              (0x03 << 8)       // Pin 4 output speed
#define GPIO_OSPEEDR_OSPEEDR4_LOW          (0x00 << 8)       //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR4_MEDIUM       (0x01 << 8)       //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR4_HIGH         (0x03 << 8)       //   - High speed
#define GPIO_OSPEEDR_OSPEEDR3              (0x03 << 6)       // Pin 3 output speed
#define GPIO_OSPEEDR_OSPEEDR3_LOW          (0x00 << 6)       //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR3_MEDIUM       (0x01 << 6)       //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR3_HIGH         (0x03 << 6)       //   - High speed
#define GPIO_OSPEEDR_OSPEEDR2              (0x03 << 4)       // Pin 2 output speed
#define GPIO_OSPEEDR_OSPEEDR2_LOW          (0x00 << 4)       //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR2_MEDIUM       (0x01 << 4)       //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR2_HIGH         (0x03 << 4)       //   - High speed
#define GPIO_OSPEEDR_OSPEEDR1              (0x03 << 2)       // Pin 1 output speed
#define GPIO_OSPEEDR_OSPEEDR1_LOW          (0x00 << 2)       //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR1_MEDIUM       (0x01 << 2)       //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR1_HIGH         (0x03 << 2)       //   - High speed
#define GPIO_OSPEEDR_OSPEEDR0              (0x03 << 0)       // Pin 0 output speed
#define GPIO_OSPEEDR_OSPEEDR0_LOW          (0x00 << 0)       //   - Low speed
#define GPIO_OSPEEDR_OSPEEDR0_MEDIUM       (0x01 << 0)       //   - Medium speed
#define GPIO_OSPEEDR_OSPEEDR0_HIGH         (0x03 << 0)       //   - High speed

#define GPIO_PUPDR_PUPDR15                 (0x03 << 30)      // Pin 15 pull-up/pull-down
#define GPIO_PUPDR_PUPDR15_NONE            (0x00 << 30)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR15_UP              (0x01 << 30)      //   - Pull-up
#define GPIO_PUPDR_PUPDR15_DOWN            (0x02 << 30)      //   - Pull-down
#define GPIO_PUPDR_PUPDR14                 (0x03 << 28)      // Pin 14 pull-up/pull-down
#define GPIO_PUPDR_PUPDR14_NONE            (0x00 << 28)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR14_UP              (0x01 << 28)      //   - Pull-up
#define GPIO_PUPDR_PUPDR14_DOWN            (0x02 << 28)      //   - Pull-down
#define GPIO_PUPDR_PUPDR13                 (0x03 << 26)      // Pin 13 pull-up/pull-down
#define GPIO_PUPDR_PUPDR13_NONE            (0x00 << 26)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR13_UP              (0x01 << 26)      //   - Pull-up
#define GPIO_PUPDR_PUPDR13_DOWN            (0x02 << 26)      //   - Pull-down
#define GPIO_PUPDR_PUPDR12                 (0x03 << 24)      // Pin 12 pull-up/pull-down
#define GPIO_PUPDR_PUPDR12_NONE            (0x00 << 24)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR12_UP              (0x01 << 24)      //   - Pull-up
#define GPIO_PUPDR_PUPDR12_DOWN            (0x02 << 24)      //   - Pull-down
#define GPIO_PUPDR_PUPDR11                 (0x03 << 22)      // Pin 11 pull-up/pull-down
#define GPIO_PUPDR_PUPDR11_NONE            (0x00 << 22)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR11_UP              (0x01 << 22)      //   - Pull-up
#define GPIO_PUPDR_PUPDR11_DOWN            (0x02 << 22)      //   - Pull-down
#define GPIO_PUPDR_PUPDR10                 (0x03 << 20)      // Pin 10 pull-up/pull-down
#define GPIO_PUPDR_PUPDR10_NONE            (0x00 << 20)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR10_UP              (0x01 << 20)      //   - Pull-up
#define GPIO_PUPDR_PUPDR10_DOWN            (0x02 << 20)      //   - Pull-down
#define GPIO_PUPDR_PUPDR9                  (0x03 << 18)      // Pin 9 pull-up/pull-down
#define GPIO_PUPDR_PUPDR9_NONE             (0x00 << 18)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR9_UP               (0x01 << 18)      //   - Pull-up
#define GPIO_PUPDR_PUPDR9_DOWN             (0x02 << 18)      //   - Pull-down
#define GPIO_PUPDR_PUPDR8                  (0x03 << 16)      // Pin 8 pull-up/pull-down
#define GPIO_PUPDR_PUPDR8_NONE             (0x00 << 16)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR8_UP               (0x01 << 16)      //   - Pull-up
#define GPIO_PUPDR_PUPDR8_DOWN             (0x02 << 16)     //   - Pull-down
#define GPIO_PUPDR_PUPDR7                  (0x03 << 14)      // Pin 7 pull-up/pull-down
#define GPIO_PUPDR_PUPDR7_NONE             (0x00 << 14)     //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR7_UP               (0x01 << 14)      //   - Pull-up
#define GPIO_PUPDR_PUPDR7_DOWN             (0x02 << 14)      //   - Pull-down
#define GPIO_PUPDR_PUPDR6                  (0x03 << 12)      // Pin 6 pull-up/pull-down
#define GPIO_PUPDR_PUPDR6_NONE             (0x00 << 12)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR6_UP               (0x01 << 12)      //   - Pull-up
#define GPIO_PUPDR_PUPDR6_DOWN             (0x02 << 12)      //   - Pull-down
#define GPIO_PUPDR_PUPDR5                  (0x03 << 10)      // Pin 5 pull-up/pull-down
#define GPIO_PUPDR_PUPDR5_NONE             (0x00 << 10)      //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR5_UP               (0x01 << 10)      //   - Pull-up
#define GPIO_PUPDR_PUPDR5_DOWN             (0x02 << 10)      //   - Pull-down
#define GPIO_PUPDR_PUPDR4                  (0x03 << 8)       // Pin 4 pull-up/pull-down
#define GPIO_PUPDR_PUPDR4_NONE             (0x00 << 8)       //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR4_UP               (0x01 << 8)       //   - Pull-up
#define GPIO_PUPDR_PUPDR4_DOWN             (0x02 << 8)       //   - Pull-down
#define GPIO_PUPDR_PUPDR3                  (0x03 << 6)       // Pin 3 pull-up/pull-down
#define GPIO_PUPDR_PUPDR3_NONE             (0x00 << 6)       //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR3_UP               (0x01 << 6)       //   - Pull-up
#define GPIO_PUPDR_PUPDR3_DOWN             (0x02 << 6)       //   - Pull-down
#define GPIO_PUPDR_PUPDR2                  (0x03 << 4)       // Pin 2 pull-up/pull-down
#define GPIO_PUPDR_PUPDR2_NONE             (0x00 << 4)       //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR2_UP               (0x01 << 4)       //   - Pull-up
#define GPIO_PUPDR_PUPDR2_DOWN             (0x02 << 4)       //   - Pull-down
#define GPIO_PUPDR_PUPDR1                  (0x03 << 2)       // Pin 1 pull-up/pull-down
#define GPIO_PUPDR_PUPDR1_NONE             (0x00 << 2)       //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR1_UP               (0x01 << 2)       //   - Pull-up
#define GPIO_PUPDR_PUPDR1_DOWN             (0x02 << 2)       //   - Pull-down
#define GPIO_PUPDR_PUPDR0                  (0x03 << 0)       // Pin 0 pull-up/pull-down
#define GPIO_PUPDR_PUPDR0_NONE             (0x00 << 0)       //   - No pull-up, pull-down
#define GPIO_PUPDR_PUPDR0_UP               (0x01 << 0)       //   - Pull-up
#define GPIO_PUPDR_PUPDR0_DOWN             (0x02 << 0)       //   - Pull-down

#define GPIO_IDR_IDR15                     (   1 << 15)      // Port 15 input data
#define GPIO_IDR_IDR14                     (   1 << 14)      // Port 14 input data
#define GPIO_IDR_IDR13                     (   1 << 13)      // Port 13 input data
#define GPIO_IDR_IDR12                     (   1 << 12)      // Port 12 input data
#define GPIO_IDR_IDR11                     (   1 << 11)      // Port 11 input data
#define GPIO_IDR_IDR10                     (   1 << 10)      // Port 10 input data
#define GPIO_IDR_IDR9                      (   1 << 9)       // Port 9 input data
#define GPIO_IDR_IDR8                      (   1 << 8)       // Port 8 input data
#define GPIO_IDR_IDR7                      (   1 << 7)       // Port 7 input data
#define GPIO_IDR_IDR6                      (   1 << 6)       // Port 6 input data
#define GPIO_IDR_IDR5                      (   1 << 5)       // Port 5 input data
#define GPIO_IDR_IDR4                      (   1 << 4)       // Port 4 input data
#define GPIO_IDR_IDR3                      (   1 << 3)       // Port 3 input data
#define GPIO_IDR_IDR2                      (   1 << 2)       // Port 2 input data
#define GPIO_IDR_IDR1                      (   1 << 1)       // Port 1 input data
#define GPIO_IDR_IDR0                      (   1 << 0)       // Port 0 input data

#define GPIO_ODR_ODR15                     (   1 << 15)      // Port 15 output data
#define GPIO_ODR_ODR14                     (   1 << 14)      // Port 14 output data
#define GPIO_ODR_ODR13                     (   1 << 13)      // Port 13 output data
#define GPIO_ODR_ODR12                     (   1 << 12)      // Port 12 output data
#define GPIO_ODR_ODR11                     (   1 << 11)      // Port 11 output data
#define GPIO_ODR_ODR10                     (   1 << 10)      // Port 10 output data
#define GPIO_ODR_ODR9                      (   1 << 9)       // Port 9 output data
#define GPIO_ODR_ODR8                      (   1 << 8)       // Port 8 output data
#define GPIO_ODR_ODR7                      (   1 << 7)       // Port 7 output data
#define GPIO_ODR_ODR6                      (   1 << 6)       // Port 6 output data
#define GPIO_ODR_ODR5                      (   1 << 5)       // Port 5 output data
#define GPIO_ODR_ODR4                      (   1 << 4)       // Port 4 output data
#define GPIO_ODR_ODR3                      (   1 << 3)       // Port 3 output data
#define GPIO_ODR_ODR2                      (   1 << 2)       // Port 2 output data
#define GPIO_ODR_ODR1                      (   1 << 1)       // Port 1 output data
#define GPIO_ODR_ODR0                      (   1 << 0)       // Port 0 output data

#define GPIO_BSRR_BR15                     (   1 << 31)      // Port 15 reset bit
#define GPIO_BSRR_BR14                     (   1 << 30)      // Port 14 reset bit
#define GPIO_BSRR_BR13                     (   1 << 29)      // Port 13 reset bit
#define GPIO_BSRR_BR12                     (   1 << 28)      // Port 12 reset bit
#define GPIO_BSRR_BR11                     (   1 << 27)      // Port 11 reset bit
#define GPIO_BSRR_BR10                     (   1 << 26)      // Port 10 reset bit
#define GPIO_BSRR_BR9                      (   1 << 25)      // Port 9 reset bit
#define GPIO_BSRR_BR8                      (   1 << 24)      // Port 8 reset bit
#define GPIO_BSRR_BR7                      (   1 << 23)      // Port 7 reset bit
#define GPIO_BSRR_BR6                      (   1 << 22)      // Port 6 reset bit
#define GPIO_BSRR_BR5                      (   1 << 21)      // Port 5 reset bit
#define GPIO_BSRR_BR4                      (   1 << 20)      // Port 4 reset bit
#define GPIO_BSRR_BR3                      (   1 << 19)      // Port 3 reset bit
#define GPIO_BSRR_BR2                      (   1 << 18)      // Port 2 reset bit
#define GPIO_BSRR_BR1                      (   1 << 17)      // Port 1 reset bit
#define GPIO_BSRR_BR0                      (   1 << 16)      // Port 0 reset bit
#define GPIO_BSRR_BS15                     (   1 << 15)      // Port 15 set bit
#define GPIO_BSRR_BS14                     (   1 << 14)      // Port 14 set bit
#define GPIO_BSRR_BS13                     (   1 << 13)      // Port 13 set bit
#define GPIO_BSRR_BS12                     (   1 << 12)      // Port 12 set bit
#define GPIO_BSRR_BS11                     (   1 << 11)      // Port 11 set bit
#define GPIO_BSRR_BS10                     (   1 << 10)      // Port 10 set bit
#define GPIO_BSRR_BS9                      (   1 << 9)       // Port 9 set bit
#define GPIO_BSRR_BS8                      (   1 << 8)       // Port 8 set bit
#define GPIO_BSRR_BS7                      (   1 << 7)       // Port 7 set bit
#define GPIO_BSRR_BS6                      (   1 << 6)       // Port 6 set bit
#define GPIO_BSRR_BS5                      (   1 << 5)       // Port 5 set bit
#define GPIO_BSRR_BS4                      (   1 << 4)       // Port 4 set bit
#define GPIO_BSRR_BS3                      (   1 << 3)       // Port 3 set bit
#define GPIO_BSRR_BS2                      (   1 << 2)       // Port 2 set bit
#define GPIO_BSRR_BS1                      (   1 << 1)       // Port 1 set bit
#define GPIO_BSRR_BS0                      (   1 << 0)       // Port 0 set bit

#define GPIO_LCKR_LCKK                     (   1 << 16)      // Lock key
#define GPIO_LCKR_LCK15                    (   1 << 15)      // Port 15 lock bit
#define GPIO_LCKR_LCK14                    (   1 << 14)      // Port 14 lock bit
#define GPIO_LCKR_LCK13                    (   1 << 13)      // Port 13 lock bit
#define GPIO_LCKR_LCK12                    (   1 << 12)      // Port 12 lock bit
#define GPIO_LCKR_LCK11                    (   1 << 11)      // Port 11 lock bit
#define GPIO_LCKR_LCK10                    (   1 << 10)      // Port 10 lock bit
#define GPIO_LCKR_LCK9                     (   1 << 9)       // Port 9 lock bit
#define GPIO_LCKR_LCK8                     (   1 << 8)       // Port 8 lock bit
#define GPIO_LCKR_LCK7                     (   1 << 7)       // Port 7 lock bit
#define GPIO_LCKR_LCK6                     (   1 << 6)       // Port 6 lock bit
#define GPIO_LCKR_LCK5                     (   1 << 5)       // Port 5 lock bit
#define GPIO_LCKR_LCK4                     (   1 << 4)       // Port 4 lock bit
#define GPIO_LCKR_LCK3                     (   1 << 3)       // Port 3 lock bit
#define GPIO_LCKR_LCK2                     (   1 << 2)       // Port 2 lock bit
#define GPIO_LCKR_LCK1                     (   1 << 1)       // Port 1 lock bit
#define GPIO_LCKR_LCK0                     (   1 << 0)       // Port 0 lock bit

#define GPIO_AFRL_AFR7                     (0x0F << 28)      // Alternate function selection for pin 7
#define GPIO_AFRL_AFR6                     (0x0F << 24)      // Alternate function selection for pin 6
#define GPIO_AFRL_AFR5                     (0x0F << 20)      // Alternate function selection for pin 5
#define GPIO_AFRL_AFR4                     (0x0F << 16)      // Alternate function selection for pin 4
#define GPIO_AFRL_AFR3                     (0x0F << 12)      // Alternate function selection for pin 3
#define GPIO_AFRL_AFR2                     (0x0F << 8)       // Alternate function selection for pin 2
#define GPIO_AFRL_AFR1                     (0x0F << 4)       // Alternate function selection for pin 1
#define GPIO_AFRL_AFR0                     (0x0F << 0)       // Alternate function selection for pin 0

#define GPIO_AFRH_AFR15                    (0x0F << 28)      // Alternate function selection for pin 15
#define GPIO_AFRH_AFR14                    (0x0F << 24)      // Alternate function selection for pin 14
#define GPIO_AFRH_AFR13                    (0x0F << 20)      // Alternate function selection for pin 13
#define GPIO_AFRH_AFR12                    (0x0F << 16)      // Alternate function selection for pin 12
#define GPIO_AFRH_AFR11                    (0x0F << 12)      // Alternate function selection for pin 11
#define GPIO_AFRH_AFR10                    (0x0F << 8)       // Alternate function selection for pin 10
#define GPIO_AFRH_AFR9                     (0x0F << 4)       // Alternate function selection for pin 9
#define GPIO_AFRH_AFR8                     (0x0F << 0)       // Alternate function selection for pin 8

#define GPIO_BRR_BR15                      (   1 << 15)      // Port 15 reset bit
#define GPIO_BRR_BR14                      (   1 << 14)      // Port 14 reset bit
#define GPIO_BRR_BR13                      (   1 << 13)      // Port 13 reset bit
#define GPIO_BRR_BR12                      (   1 << 12)      // Port 12 reset bit
#define GPIO_BRR_BR11                      (   1 << 11)      // Port 11 reset bit
#define GPIO_BRR_BR10                      (   1 << 10)      // Port 10 reset bit
#define GPIO_BRR_BR9                       (   1 << 9)       // Port 9 reset bit
#define GPIO_BRR_BR8                       (   1 << 8)       // Port 8 reset bit
#define GPIO_BRR_BR7                       (   1 << 7)       // Port 7 reset bit
#define GPIO_BRR_BR6                       (   1 << 6)       // Port 6 reset bit
#define GPIO_BRR_BR5                       (   1 << 5)       // Port 5 reset bit
#define GPIO_BRR_BR4                       (   1 << 4)       // Port 4 reset bit
#define GPIO_BRR_BR3                       (   1 << 3)       // Port 3 reset bit
#define GPIO_BRR_BR2                       (   1 << 2)       // Port 2 reset bit
#define GPIO_BRR_BR1                       (   1 << 1)       // Port 1 reset bit
#define GPIO_BRR_BR0                       (   1 << 0)       // Port 0 reset bit

// == SYSCFG ==========================================================================================================

struct SYSCFG_t {
	_RW uint32_t CFGR1;          // Configuration register 1
	_RS uint32_t reserved1;
	_RW uint32_t EXTICR1;        // External interrupt configuration register 1
	_RW uint32_t EXTICR2;        // External interrupt configuration register 2
	_RW uint32_t EXTICR3;        // External interrupt configuration register 3
	_RW uint32_t EXTICR4;        // External interrupt configuration register 4
	_RW uint32_t CFGR2;          // Configuration register 2
};
static_assert(offsetof(struct SYSCFG_t, CFGR2) == 0x18, "Wrong definition");

#define SYSCFG_CFGR1_USART3DMARMP          (   1 << 26)      // USART3 DMA request remapping bit
#define SYSCFG_CFGR1_I2CPA10FMP            (   1 << 23)      // Fast Mode Plus (FM+) driving capability activation bit
#define SYSCFG_CFGR1_I2CPA9FMP             (   1 << 22)      // Fast Mode Plus (FM+) driving capability activation bit
#define SYSCFG_CFGR1_I2C1_FMP              (   1 << 20)      // FM+ driving capability activation for I2C1
#define SYSCFG_CFGR1_I2CPB9FMP             (   1 << 19)      // Fast Mode Plus (FM+) driving capability activation bit
#define SYSCFG_CFGR1_I2CPB8FMP             (   1 << 18)      // Fast Mode Plus (FM+) driving capability activation bit
#define SYSCFG_CFGR1_I2CPB7FMP             (   1 << 17)      // Fast Mode Plus (FM+) driving capability activation bit
#define SYSCFG_CFGR1_I2CPB6FMP             (   1 << 16)      // Fast Mode Plus (FM+) driving capability activation bit
#define SYSCFG_CFGR1_TIM17DMARMP           (   1 << 12)      // TIM17 DMA request remapping bit
#define SYSCFG_CFGR1_TIM16DMARMP           (   1 << 11)      // TIM16 DMA request remapping bit
#define SYSCFG_CFGR1_USART1RXDMARMP        (   1 << 10)      // USART1_RX DMA request remapping bit
#define SYSCFG_CFGR1_USART1TXDMARMP        (   1 << 9)       // USART1_TX DMA request remapping bit
#define SYSCFG_CFGR1_ADCDMARMP             (   1 << 8)       // ADC DMA request remapping bit
#define SYSCFG_CFGR1_PA11PA12RMP           (   1 << 4)       // PA11 and PA12 remapping bit for small packages
#define SYSCFG_CFGR1_MEMMODE               (0x03 << 0)       // Memory mapping selection bits
#define SYSCFG_CFGR1_MEMMODE_MAIN          (0x00 << 0)       //   - Main Flash memory mapped at 0x0000 0000
#define SYSCFG_CFGR1_MEMMODE_SYSTEM        (0x01 << 0)       //   - System Flash memory mapped at 0x0000 0000
#define SYSCFG_CFGR1_MEMMODE_SRAM          (0x03 << 0)       //   - Embedded SRAM mapped at 0x0000 0000

#define SYSCFG_EXTICR1_EXTI3               (0x0F << 12)      // EXTI 3 configuration bits
#define SYSCFG_EXTICR1_EXTI3_PA3           (0x00 << 12)      //   - PA3
#define SYSCFG_EXTICR1_EXTI3_PB3           (0x01 << 12)      //   - PB3
#define SYSCFG_EXTICR1_EXTI3_PC3           (0x02 << 12)      //   - PC3
#define SYSCFG_EXTICR1_EXTI3_PD3           (0x03 << 12)      //   - PD3
#define SYSCFG_EXTICR1_EXTI3_PF3           (0x07 << 12)      //   - PF3
#define SYSCFG_EXTICR1_EXTI2               (0x0F << 8)       // EXTI 2 configuration bits
#define SYSCFG_EXTICR1_EXTI2_PA2           (0x00 << 8)       //   - PA2
#define SYSCFG_EXTICR1_EXTI2_PB2           (0x01 << 8)       //   - PB2
#define SYSCFG_EXTICR1_EXTI2_PC2           (0x02 << 8)       //   - PC2
#define SYSCFG_EXTICR1_EXTI2_PD2           (0x03 << 8)       //   - PD2
#define SYSCFG_EXTICR1_EXTI2_PF2           (0x07 << 8)       //   - PF2
#define SYSCFG_EXTICR1_EXTI1               (0x0F << 4)       // EXTI 1 configuration bits
#define SYSCFG_EXTICR1_EXTI1_PA1           (0x00 << 4)       //   - PA1
#define SYSCFG_EXTICR1_EXTI1_PB1           (0x01 << 4)       //   - PB1
#define SYSCFG_EXTICR1_EXTI1_PC1           (0x02 << 4)       //   - PC1
#define SYSCFG_EXTICR1_EXTI1_PD1           (0x03 << 4)       //   - PD1
#define SYSCFG_EXTICR1_EXTI1_PF1           (0x07 << 4)       //   - PF1
#define SYSCFG_EXTICR1_EXTI0               (0x0F << 0)       // EXTI 0 configuration bits
#define SYSCFG_EXTICR1_EXTI0_PA0           (0x00 << 0)       //   - PA0
#define SYSCFG_EXTICR1_EXTI0_PB0           (0x01 << 0)       //   - PB0
#define SYSCFG_EXTICR1_EXTI0_PC0           (0x02 << 0)       //   - PC0
#define SYSCFG_EXTICR1_EXTI0_PD0           (0x03 << 0)       //   - PD0
#define SYSCFG_EXTICR1_EXTI0_PF0           (0x07 << 0)       //   - PF0

#define SYSCFG_EXTICR2_EXTI7               (0x0F << 12)      // EXTI 7 configuration bits
#define SYSCFG_EXTICR2_EXTI7_PA7           (0x00 << 12)      //   - PA7
#define SYSCFG_EXTICR2_EXTI7_PB7           (0x01 << 12)      //   - PB7
#define SYSCFG_EXTICR2_EXTI7_PC7           (0x02 << 12)      //   - PC7
#define SYSCFG_EXTICR2_EXTI7_PD7           (0x03 << 12)      //   - PD7
#define SYSCFG_EXTICR2_EXTI7_PF7           (0x07 << 12)      //   - PF7
#define SYSCFG_EXTICR2_EXTI6               (0x0F << 8)       // EXTI 6 configuration bits
#define SYSCFG_EXTICR2_EXTI6_PA6           (0x00 << 8)       //   - PA6
#define SYSCFG_EXTICR2_EXTI6_PB6           (0x01 << 8)       //   - PB6
#define SYSCFG_EXTICR2_EXTI6_PC6           (0x02 << 8)       //   - PC6
#define SYSCFG_EXTICR2_EXTI6_PD6           (0x03 << 8)       //   - PD6
#define SYSCFG_EXTICR2_EXTI6_PF6           (0x07 << 8)       //   - PF6
#define SYSCFG_EXTICR2_EXTI5               (0x0F << 4)       // EXTI 5 configuration bits
#define SYSCFG_EXTICR2_EXTI5_PA5           (0x00 << 4)       //   - PA5
#define SYSCFG_EXTICR2_EXTI5_PB5           (0x01 << 4)       //   - PB5
#define SYSCFG_EXTICR2_EXTI5_PC5           (0x02 << 4)       //   - PC5
#define SYSCFG_EXTICR2_EXTI5_PD5           (0x03 << 4)       //   - PD5
#define SYSCFG_EXTICR2_EXTI5_PF5           (0x07 << 4)       //   - PF5
#define SYSCFG_EXTICR2_EXTI4               (0x0F << 0)       // EXTI 4 configuration bits
#define SYSCFG_EXTICR2_EXTI4_PA4           (0x00 << 0)       //   - PA4
#define SYSCFG_EXTICR2_EXTI4_PB4           (0x01 << 0)       //   - PB4
#define SYSCFG_EXTICR2_EXTI4_PC4           (0x02 << 0)       //   - PC4
#define SYSCFG_EXTICR2_EXTI4_PD4           (0x03 << 0)       //   - PD4
#define SYSCFG_EXTICR2_EXTI4_PF4           (0x07 << 0)       //   - PF4

#define SYSCFG_EXTICR3_EXTI11              (0x0F << 12)      // EXTI 11 configuration bits
#define SYSCFG_EXTICR3_EXTI11_PA11         (0x00 << 12)      //   - PA11
#define SYSCFG_EXTICR3_EXTI11_PB11         (0x01 << 12)      //   - PB11
#define SYSCFG_EXTICR3_EXTI11_PC11         (0x02 << 12)      //   - PC11
#define SYSCFG_EXTICR3_EXTI11_PD11         (0x03 << 12)      //   - PD11
#define SYSCFG_EXTICR3_EXTI11_PF11         (0x07 << 12)      //   - PF11
#define SYSCFG_EXTICR3_EXTI10              (0x0F << 8)       // EXTI 10 configuration bits
#define SYSCFG_EXTICR3_EXTI10_PA10         (0x00 << 8)       //   - PA10
#define SYSCFG_EXTICR3_EXTI10_PB10         (0x01 << 8)       //   - PB10
#define SYSCFG_EXTICR3_EXTI10_PC10         (0x02 << 8)       //   - PC10
#define SYSCFG_EXTICR3_EXTI10_PD10         (0x03 << 8)       //   - PD10
#define SYSCFG_EXTICR3_EXTI10_PF10         (0x07 << 8)       //   - PF10
#define SYSCFG_EXTICR3_EXTI9               (0x0F << 4)       // EXTI 9 configuration bits
#define SYSCFG_EXTICR3_EXTI9_PA9           (0x00 << 4)       //   - PA9
#define SYSCFG_EXTICR3_EXTI9_PB9           (0x01 << 4)       //   - PB9
#define SYSCFG_EXTICR3_EXTI9_PC9           (0x02 << 4)       //   - PC9
#define SYSCFG_EXTICR3_EXTI9_PD9           (0x03 << 4)       //   - PD9
#define SYSCFG_EXTICR3_EXTI9_PF9           (0x07 << 4)       //   - PF9
#define SYSCFG_EXTICR3_EXTI8               (0x0F << 0)       // EXTI 8 configuration bits
#define SYSCFG_EXTICR3_EXTI8_PA8           (0x00 << 0)       //   - PA8
#define SYSCFG_EXTICR3_EXTI8_PB8           (0x01 << 0)       //   - PB8
#define SYSCFG_EXTICR3_EXTI8_PC8           (0x02 << 0)       //   - PC8
#define SYSCFG_EXTICR3_EXTI8_PD8           (0x03 << 0)       //   - PD8
#define SYSCFG_EXTICR3_EXTI8_PF8           (0x07 << 0)       //   - PF8

#define SYSCFG_EXTICR4_EXTI15              (0x0F << 12)      // EXTI 15 configuration bits
#define SYSCFG_EXTICR4_EXTI15_PA15         (0x00 << 12)      //   - PA15
#define SYSCFG_EXTICR4_EXTI15_PB15         (0x01 << 12)      //   - PB15
#define SYSCFG_EXTICR4_EXTI15_PC15         (0x02 << 12)      //   - PC15
#define SYSCFG_EXTICR4_EXTI15_PD15         (0x03 << 12)      //   - PD15
#define SYSCFG_EXTICR4_EXTI15_PF15         (0x07 << 12)      //   - PF15
#define SYSCFG_EXTICR4_EXTI14              (0x0F << 8)       // EXTI 14 configuration bits
#define SYSCFG_EXTICR4_EXTI14_PA14         (0x00 << 8)       //   - PA14
#define SYSCFG_EXTICR4_EXTI14_PB14         (0x01 << 8)       //   - PB14
#define SYSCFG_EXTICR4_EXTI14_PC14         (0x02 << 8)       //   - PC14
#define SYSCFG_EXTICR4_EXTI14_PD14         (0x03 << 8)       //   - PD14
#define SYSCFG_EXTICR4_EXTI14_PF14         (0x07 << 8)       //   - PF14
#define SYSCFG_EXTICR4_EXTI13              (0x0F << 4)       // EXTI 13 configuration bits
#define SYSCFG_EXTICR4_EXTI13_PA13         (0x00 << 4)       //   - PA13
#define SYSCFG_EXTICR1_EXTI13_PB13         (0x01 << 4)       //   - PB13
#define SYSCFG_EXTICR4_EXTI13_PC13         (0x02 << 4)       //   - PC13
#define SYSCFG_EXTICR4_EXTI13_PD13         (0x03 << 4)       //   - PD13
#define SYSCFG_EXTICR4_EXTI13_PF13         (0x07 << 4)       //   - PF13
#define SYSCFG_EXTICR4_EXTI12              (0x0F << 0)       // EXTI 12 configuration bits
#define SYSCFG_EXTICR4_EXTI12_PA12         (0x00 << 0)       //   - PA12
#define SYSCFG_EXTICR4_EXTI12_PB12         (0x01 << 0)       //   - PB12
#define SYSCFG_EXTICR4_EXTI12_PC12         (0x02 << 0)       //   - PC12
#define SYSCFG_EXTICR4_EXTI12_PD12         (0x03 << 0)       //   - PD12
#define SYSCFG_EXTICR4_EXTI12_PF12         (0x07 << 0)       //   - PF12

#define SYSCFG_CFGR2_SRAMPEF               (   1 << 8)       // SRAM parity error flag
#define SYSCFG_CFGR2_SRAMPARITYLOCK        (   1 << 1)       // SRAM parity lock bit
#define SYSCFG_CFGR2_LOCKUPLOCK            (   1 << 0)       // Cortex-M0 LOCKUP bit enable bit

// == EXTI ============================================================================================================

struct EXTI_t {
	_RW uint32_t IMR;            // Interrupt mask register
	_RW uint32_t EMR;            // Event mask register
	_RW uint32_t RTSR;           // Rising trigger selection register
	_RW uint32_t FTSR;           // Falling trigger selection register
	_RW uint32_t SWIER;          // Software interrupt event register
	_RW uint32_t PR;             // Pending register
};
static_assert(offsetof(struct EXTI_t, PR) == 0x14, "Wrong definition");

#define EXTI_IMR_MR31                      (   1 << 31)      // Interrupt Mask on line 31
#define EXTI_IMR_MR30                      (   1 << 30)      // Interrupt Mask on line 30
#define EXTI_IMR_MR29                      (   1 << 29)      // Interrupt Mask on line 29
#define EXTI_IMR_MR28                      (   1 << 28)      // Interrupt Mask on line 28
#define EXTI_IMR_MR27                      (   1 << 27)      // Interrupt Mask on line 27
#define EXTI_IMR_MR26                      (   1 << 26)      // Interrupt Mask on line 26
#define EXTI_IMR_MR25                      (   1 << 25)      // Interrupt Mask on line 25
#define EXTI_IMR_MR24                      (   1 << 24)      // Interrupt Mask on line 24
#define EXTI_IMR_MR23                      (   1 << 23)      // Interrupt Mask on line 23
#define EXTI_IMR_MR22                      (   1 << 22)      // Interrupt Mask on line 22
#define EXTI_IMR_MR21                      (   1 << 21)      // Interrupt Mask on line 21
#define EXTI_IMR_MR20                      (   1 << 20)      // Interrupt Mask on line 20
#define EXTI_IMR_MR19                      (   1 << 19)      // Interrupt Mask on line 19
#define EXTI_IMR_MR18                      (   1 << 18)      // Interrupt Mask on line 18
#define EXTI_IMR_MR17                      (   1 << 17)      // Interrupt Mask on line 17
#define EXTI_IMR_MR16                      (   1 << 16)      // Interrupt Mask on line 16
#define EXTI_IMR_MR15                      (   1 << 15)      // Interrupt Mask on line 15
#define EXTI_IMR_MR14                      (   1 << 14)      // Interrupt Mask on line 14
#define EXTI_IMR_MR13                      (   1 << 13)      // Interrupt Mask on line 13
#define EXTI_IMR_MR12                      (   1 << 12)      // Interrupt Mask on line 12
#define EXTI_IMR_MR11                      (   1 << 11)      // Interrupt Mask on line 11
#define EXTI_IMR_MR10                      (   1 << 10)      // Interrupt Mask on line 10
#define EXTI_IMR_MR9                       (   1 << 9)       // Interrupt Mask on line 9
#define EXTI_IMR_MR8                       (   1 << 8)       // Interrupt Mask on line 8
#define EXTI_IMR_MR7                       (   1 << 7)       // Interrupt Mask on line 7
#define EXTI_IMR_MR6                       (   1 << 6)       // Interrupt Mask on line 6
#define EXTI_IMR_MR5                       (   1 << 5)       // Interrupt Mask on line 5
#define EXTI_IMR_MR4                       (   1 << 4)       // Interrupt Mask on line 4
#define EXTI_IMR_MR3                       (   1 << 3)       // Interrupt Mask on line 3
#define EXTI_IMR_MR2                       (   1 << 2)       // Interrupt Mask on line 2
#define EXTI_IMR_MR1                       (   1 << 1)       // Interrupt Mask on line 1
#define EXTI_IMR_MR0                       (   1 << 0)       // Interrupt Mask on line 0

#define EXTI_EMR_MR31                      (   1 << 31)      // Event Mask on line 31
#define EXTI_EMR_MR30                      (   1 << 30)      // Event Mask on line 30
#define EXTI_EMR_MR29                      (   1 << 29)      // Event Mask on line 29
#define EXTI_EMR_MR28                      (   1 << 28)      // Event Mask on line 28
#define EXTI_EMR_MR27                      (   1 << 27)      // Event Mask on line 27
#define EXTI_EMR_MR26                      (   1 << 26)      // Event Mask on line 26
#define EXTI_EMR_MR25                      (   1 << 25)      // Event Mask on line 25
#define EXTI_EMR_MR24                      (   1 << 24)      // Event Mask on line 24
#define EXTI_EMR_MR23                      (   1 << 23)      // Event Mask on line 23
#define EXTI_EMR_MR22                      (   1 << 22)      // Event Mask on line 22
#define EXTI_EMR_MR21                      (   1 << 21)      // Event Mask on line 21
#define EXTI_EMR_MR20                      (   1 << 20)      // Event Mask on line 20
#define EXTI_EMR_MR19                      (   1 << 19)      // Event Mask on line 19
#define EXTI_EMR_MR18                      (   1 << 18)      // Event Mask on line 18
#define EXTI_EMR_MR17                      (   1 << 17)      // Event Mask on line 17
#define EXTI_EMR_MR16                      (   1 << 16)      // Event Mask on line 16
#define EXTI_EMR_MR15                      (   1 << 15)      // Event Mask on line 15
#define EXTI_EMR_MR14                      (   1 << 14)      // Event Mask on line 14
#define EXTI_EMR_MR13                      (   1 << 13)      // Event Mask on line 13
#define EXTI_EMR_MR12                      (   1 << 12)      // Event Mask on line 12
#define EXTI_EMR_MR11                      (   1 << 11)      // Event Mask on line 11
#define EXTI_EMR_MR10                      (   1 << 10)      // Event Mask on line 10
#define EXTI_EMR_MR9                       (   1 << 9)       // Event Mask on line 9
#define EXTI_EMR_MR8                       (   1 << 8)       // Event Mask on line 8
#define EXTI_EMR_MR7                       (   1 << 7)       // Event Mask on line 7
#define EXTI_EMR_MR6                       (   1 << 6)       // Event Mask on line 6
#define EXTI_EMR_MR5                       (   1 << 5)       // Event Mask on line 5
#define EXTI_EMR_MR4                       (   1 << 4)       // Event Mask on line 4
#define EXTI_EMR_MR3                       (   1 << 3)       // Event Mask on line 3
#define EXTI_EMR_MR2                       (   1 << 2)       // Event Mask on line 2
#define EXTI_EMR_MR1                       (   1 << 1)       // Event Mask on line 1
#define EXTI_EMR_MR0                       (   1 << 0)       // Event Mask on line 0

#define EXTI_RTSR_TR22                     (   1 << 22)      // Rising trigger event configuration bit of line 22
#define EXTI_RTSR_TR21                     (   1 << 21)      // Rising trigger event configuration bit of line 21
#define EXTI_RTSR_TR20                     (   1 << 20)      // Rising trigger event configuration bit of line 20
#define EXTI_RTSR_TR19                     (   1 << 19)      // Rising trigger event configuration bit of line 19
#define EXTI_RTSR_TR17                     (   1 << 17)      // Rising trigger event configuration bit of line 17
#define EXTI_RTSR_TR16                     (   1 << 16)      // Rising trigger event configuration bit of line 16
#define EXTI_RTSR_TR15                     (   1 << 15)      // Rising trigger event configuration bit of line 15
#define EXTI_RTSR_TR14                     (   1 << 14)      // Rising trigger event configuration bit of line 14
#define EXTI_RTSR_TR13                     (   1 << 13)      // Rising trigger event configuration bit of line 13
#define EXTI_RTSR_TR12                     (   1 << 12)      // Rising trigger event configuration bit of line 12
#define EXTI_RTSR_TR11                     (   1 << 11)      // Rising trigger event configuration bit of line 11
#define EXTI_RTSR_TR10                     (   1 << 10)      // Rising trigger event configuration bit of line 10
#define EXTI_RTSR_TR9                      (   1 << 9)       // Rising trigger event configuration bit of line 9
#define EXTI_RTSR_TR8                      (   1 << 8)       // Rising trigger event configuration bit of line 8
#define EXTI_RTSR_TR7                      (   1 << 7)       // Rising trigger event configuration bit of line 7
#define EXTI_RTSR_TR6                      (   1 << 6)       // Rising trigger event configuration bit of line 6
#define EXTI_RTSR_TR5                      (   1 << 5)       // Rising trigger event configuration bit of line 5
#define EXTI_RTSR_TR4                      (   1 << 4)       // Rising trigger event configuration bit of line 4
#define EXTI_RTSR_TR3                      (   1 << 3)       // Rising trigger event configuration bit of line 3
#define EXTI_RTSR_TR2                      (   1 << 2)       // Rising trigger event configuration bit of line 2
#define EXTI_RTSR_TR1                      (   1 << 1)       // Rising trigger event configuration bit of line 1
#define EXTI_RTSR_TR0                      (   1 << 0)       // Rising trigger event configuration bit of line 0

#define EXTI_FTSR_TR22                     (   1 << 22)      // Falling trigger event configuration bit of line 22
#define EXTI_FTSR_TR21                     (   1 << 21)      // Falling trigger event configuration bit of line 21
#define EXTI_FTSR_TR20                     (   1 << 20)      // Falling trigger event configuration bit of line 20
#define EXTI_FTSR_TR19                     (   1 << 19)      // Falling trigger event configuration bit of line 19
#define EXTI_FTSR_TR17                     (   1 << 17)      // Falling trigger event configuration bit of line 17
#define EXTI_FTSR_TR16                     (   1 << 16)      // Falling trigger event configuration bit of line 16
#define EXTI_FTSR_TR15                     (   1 << 15)      // Falling trigger event configuration bit of line 15
#define EXTI_FTSR_TR14                     (   1 << 14)      // Falling trigger event configuration bit of line 14
#define EXTI_FTSR_TR13                     (   1 << 13)      // Falling trigger event configuration bit of line 13
#define EXTI_FTSR_TR12                     (   1 << 12)      // Falling trigger event configuration bit of line 12
#define EXTI_FTSR_TR11                     (   1 << 11)      // Falling trigger event configuration bit of line 11
#define EXTI_FTSR_TR10                     (   1 << 10)      // Falling trigger event configuration bit of line 10
#define EXTI_FTSR_TR9                      (   1 << 9)       // Falling trigger event configuration bit of line 9
#define EXTI_FTSR_TR8                      (   1 << 8)       // Falling trigger event configuration bit of line 8
#define EXTI_FTSR_TR7                      (   1 << 7)       // Falling trigger event configuration bit of line 7
#define EXTI_FTSR_TR6                      (   1 << 6)       // Falling trigger event configuration bit of line 6
#define EXTI_FTSR_TR5                      (   1 << 5)       // Falling trigger event configuration bit of line 5
#define EXTI_FTSR_TR4                      (   1 << 4)       // Falling trigger event configuration bit of line 4
#define EXTI_FTSR_TR3                      (   1 << 3)       // Falling trigger event configuration bit of line 3
#define EXTI_FTSR_TR2                      (   1 << 2)       // Falling trigger event configuration bit of line 2
#define EXTI_FTSR_TR1                      (   1 << 1)       // Falling trigger event configuration bit of line 1
#define EXTI_FTSR_TR0                      (   1 << 0)       // Falling trigger event configuration bit of line 0

#define EXTI_SWIER_SWIER22                 (   1 << 22)      // Software interrupt on line 22
#define EXTI_SWIER_SWIER21                 (   1 << 21)      // Software interrupt on line 21
#define EXTI_SWIER_SWIER20                 (   1 << 20)      // Software interrupt on line 20
#define EXTI_SWIER_SWIER19                 (   1 << 19)      // Software interrupt on line 19
#define EXTI_SWIER_SWIER17                 (   1 << 17)      // Software interrupt on line 17
#define EXTI_SWIER_SWIER16                 (   1 << 16)      // Software interrupt on line 16
#define EXTI_SWIER_SWIER15                 (   1 << 15)      // Software interrupt on line 15
#define EXTI_SWIER_SWIER14                 (   1 << 14)      // Software interrupt on line 14
#define EXTI_SWIER_SWIER13                 (   1 << 13)      // Software interrupt on line 13
#define EXTI_SWIER_SWIER12                 (   1 << 12)      // Software interrupt on line 12
#define EXTI_SWIER_SWIER11                 (   1 << 11)      // Software interrupt on line 11
#define EXTI_SWIER_SWIER10                 (   1 << 10)      // Software interrupt on line 10
#define EXTI_SWIER_SWIER9                  (   1 << 9)       // Software interrupt on line 9
#define EXTI_SWIER_SWIER8                  (   1 << 8)       // Software interrupt on line 8
#define EXTI_SWIER_SWIER7                  (   1 << 7)       // Software interrupt on line 7
#define EXTI_SWIER_SWIER6                  (   1 << 6)       // Software interrupt on line 6
#define EXTI_SWIER_SWIER5                  (   1 << 5)       // Software interrupt on line 5
#define EXTI_SWIER_SWIER4                  (   1 << 4)       // Software interrupt on line 4
#define EXTI_SWIER_SWIER3                  (   1 << 3)       // Software interrupt on line 3
#define EXTI_SWIER_SWIER2                  (   1 << 2)       // Software interrupt on line 2
#define EXTI_SWIER_SWIER1                  (   1 << 1)       // Software interrupt on line 1
#define EXTI_SWIER_SWIER0                  (   1 << 0)       // Software interrupt on line 0

#define EXTI_PR_PR22                       (   1 << 22)      // Pending bit for line 22
#define EXTI_PR_PR21                       (   1 << 21)      // Pending bit for line 21
#define EXTI_PR_PR20                       (   1 << 20)      // Pending bit for line 20
#define EXTI_PR_PR19                       (   1 << 19)      // Pending bit for line 19
#define EXTI_PR_PR17                       (   1 << 17)      // Pending bit for line 17
#define EXTI_PR_PR16                       (   1 << 16)      // Pending bit for line 16
#define EXTI_PR_PR15                       (   1 << 15)      // Pending bit for line 15
#define EXTI_PR_PR14                       (   1 << 14)      // Pending bit for line 14
#define EXTI_PR_PR13                       (   1 << 13)      // Pending bit for line 13
#define EXTI_PR_PR12                       (   1 << 12)      // Pending bit for line 12
#define EXTI_PR_PR11                       (   1 << 11)      // Pending bit for line 11
#define EXTI_PR_PR10                       (   1 << 10)      // Pending bit for line 10
#define EXTI_PR_PR9                        (   1 << 9)       // Pending bit for line 9
#define EXTI_PR_PR8                        (   1 << 8)       // Pending bit for line 8
#define EXTI_PR_PR7                        (   1 << 7)       // Pending bit for line 7
#define EXTI_PR_PR6                        (   1 << 6)       // Pending bit for line 6
#define EXTI_PR_PR5                        (   1 << 5)       // Pending bit for line 5
#define EXTI_PR_PR4                        (   1 << 4)       // Pending bit for line 4
#define EXTI_PR_PR3                        (   1 << 3)       // Pending bit for line 3
#define EXTI_PR_PR2                        (   1 << 2)       // Pending bit for line 2
#define EXTI_PR_PR1                        (   1 << 1)       // Pending bit for line 1
#define EXTI_PR_PR0                        (   1 << 0)       // Pending bit for line 0

// == DMA =============================================================================================================

struct DmaChannel_t {
	_RW uint32_t CCR;            // DMA channel x configuration register
	_RW uint32_t CNDTR;          // DMA channel x number of data register
	_RW uint32_t CPAR;           // DMA channel x peripheral address register
	_RW uint32_t CMAR;           // DMA channel x memory address register
	_RS uint32_t reserved;
};

struct DMA_t {
	_RO uint32_t ISR;            // Interrupt status register
	_WO uint32_t IFCR;           // Interrupt flag clear register
	struct DmaChannel_t channels[5];    // Channels 1-5
	_RS uint32_t reserved1[15];
	_RW uint32_t CSELR;          // Channel selection register (vailable on STM32F030xC devices only)
};
static_assert(offsetof(struct DMA_t, channels[4].CMAR) == 0x64, "Wrong definition");

#define DMA_ISR_TEIF5                      (   1 << 19)      // Channel 5 transfer error flag
#define DMA_ISR_HTIF5                      (   1 << 18)      // Channel 5 half transfer flag
#define DMA_ISR_TCIF5                      (   1 << 17)      // Channel 5 transfer complete flag
#define DMA_ISR_GIF5                       (   1 << 16)      // Channel 5 transfer complete flag
#define DMA_ISR_TEIF4                      (   1 << 15)      // Channel 4 transfer error flag
#define DMA_ISR_HTIF4                      (   1 << 14)      // Channel 4 half transfer flag
#define DMA_ISR_TCIF4                      (   1 << 13)      // Channel 4 transfer complete flag
#define DMA_ISR_GIF4                       (   1 << 12)      // Channel 4 transfer complete flag
#define DMA_ISR_TEIF3                      (   1 << 11)      // Channel 3 transfer error flag
#define DMA_ISR_HTIF3                      (   1 << 10)      // Channel 3 half transfer flag
#define DMA_ISR_TCIF3                      (   1 << 9)       // Channel 3 transfer complete flag
#define DMA_ISR_GIF3                       (   1 << 8)       // Channel 3 transfer complete flag
#define DMA_ISR_TEIF2                      (   1 << 7)       // Channel 2 transfer error flag
#define DMA_ISR_HTIF2                      (   1 << 6)       // Channel 2 half transfer flag
#define DMA_ISR_TCIF2                      (   1 << 5)       // Channel 2 transfer complete flag
#define DMA_ISR_GIF2                       (   1 << 4)       // Channel 2 transfer complete flag
#define DMA_ISR_TEIF1                      (   1 << 3)       // Channel 1 transfer error flag
#define DMA_ISR_HTIF1                      (   1 << 2)       // Channel 1 half transfer flag
#define DMA_ISR_TCIF1                      (   1 << 1)       // Channel 1 transfer complete flag
#define DMA_ISR_GIF1                       (   1 << 0)       // Channel 1 transfer complete flag

#define DMA_IFCR_CTEIF5                    (   1 << 19)      // Channel 5 transfer error clear
#define DMA_IFCR_CHTIF5                    (   1 << 18)      // Channel 5 half transfer clear
#define DMA_IFCR_CTCIF5                    (   1 << 17)      // Channel 5 transfer complete clear
#define DMA_IFCR_CGIF5                     (   1 << 16)      // Channel 5 global interrupt clear
#define DMA_IFCR_CTEIF4                    (   1 << 15)      // Channel 4 transfer error clear
#define DMA_IFCR_CHTIF4                    (   1 << 14)      // Channel 4 half transfer clear
#define DMA_IFCR_CTCIF4                    (   1 << 13)      // Channel 4 transfer complete clear
#define DMA_IFCR_CGIF4                     (   1 << 12)      // Channel 4 global interrupt clear
#define DMA_IFCR_CTEIF3                    (   1 << 11)      // Channel 3 transfer error clear
#define DMA_IFCR_CHTIF3                    (   1 << 10)      // Channel 3 half transfer clear
#define DMA_IFCR_CTCIF3                    (   1 << 9)       // Channel 3 transfer complete clear
#define DMA_IFCR_CGIF3                     (   1 << 8)       // Channel 3 global interrupt clear
#define DMA_IFCR_CTEIF2                    (   1 << 7)       // Channel 2 transfer error clear
#define DMA_IFCR_CHTIF2                    (   1 << 6)       // Channel 2 half transfer clear
#define DMA_IFCR_CTCIF2                    (   1 << 5)       // Channel 2 transfer complete clear
#define DMA_IFCR_CGIF2                     (   1 << 4)       // Channel 2 global interrupt clear
#define DMA_IFCR_CTEIF1                    (   1 << 3)       // Channel 1 transfer error clear
#define DMA_IFCR_CHTIF1                    (   1 << 2)       // Channel 1 half transfer clear
#define DMA_IFCR_CTCIF1                    (   1 << 1)       // Channel 1 transfer complete clear
#define DMA_IFCR_CGIF1                     (   1 << 0)       // Channel 1 global interrupt clear

#define DMA_CCR_MEM2MEM                    (   1 << 14)      // Memory to memory mode
#define DMA_CCR_PL                         (0x03 << 12)      // Channel priority level (0 - low)
#define DMA_CCR_MSIZE                      (0x03 << 10)      // Memory size
#define DMA_CCR_MSIZE_8BIT                 (0x00 << 10)      //   - 8-bits
#define DMA_CCR_MSIZE_16BIT                (0x01 << 10)      //   - 16-bits
#define DMA_CCR_MSIZE_32BIT                (0x02 << 10)      //   - 32-bits
#define DMA_CCR_PSIZE                      (0x03 << 8)       // Peripheral size
#define DMA_CCR_PSIZE_8BIT                 (0x00 << 8)       //   - 8-bits
#define DMA_CCR_PSIZE_16BIT                (0x01 << 8)       //   - 16-bits
#define DMA_CCR_PSIZE_32BIT                (0x02 << 8)       //   - 32-bits
#define DMA_CCR_MINC                       (   1 << 7)       // Memory increment mode
#define DMA_CCR_PINC                       (   1 << 6)       // Peripheral increment mode
#define DMA_CCR_CIRC                       (   1 << 5)       // Circular mode
#define DMA_CCR_DIR                        (   1 << 4)       // Data transfer direction
#define DMA_CCR_TEIE                       (   1 << 3)       // Transfer error interrupt enable
#define DMA_CCR_HTIE                       (   1 << 2)       // Half transfer interrupt enable
#define DMA_CCR_TCIE                       (   1 << 1)       // Transfer complete interrupt enable
#define DMA_CCR_EN                         (   1 << 0)       // Channel enable

#define DMA_CNDTR_NDT                      (0xFFFF << 0)     // Number of data to transfer

#define DMA_CPAR_PA                        (0xFFFFFFFF << 0) // Peripheral address

#define DMA_CMAR_MA                        (0xFFFFFFFF << 0) // Memory address

#define DMA_CSELR_C5S                      (0x0F << 16)      // DMA channel 5 selection
#define DMA_CSELR_C4S                      (0x0F << 12)      // DMA channel 4 selection
#define DMA_CSELR_C3S                      (0x0F << 8)       // DMA channel 3 selection
#define DMA_CSELR_C2S                      (0x0F << 4)       // DMA channel 2 selection
#define DMA_CSELR_C1S                      (0x0F << 0)       // DMA channel 1 selection

// == ADC =============================================================================================================

struct ADC_t {
	_RW uint32_t ISR;            // Interrupt and status register
	_RW uint32_t IER;            // Interrupt enable register
	_RW uint32_t CR;             // Control register
	_RW uint32_t CFGR1;          // Configuration register 1
	_RW uint32_t CFGR2;          // Configuration register 2
	_RW uint32_t SMPR;           // Sample time register
	_RS uint32_t reserved1[2];
	_RW uint32_t TR;             // Watchdog threshold register
	_RS uint32_t reserved2;
	_RW uint32_t CHSELR;         // Channel selection register
	_RS uint32_t reserved3[5];
	_RO uint32_t DR;             // Data register
	_RS uint32_t reserved4[177];
	_RW uint32_t CCR;            // Common configuration register
};
static_assert(offsetof(struct ADC_t, CHSELR) == 0x0028, "Wrong definition");
static_assert(offsetof(struct ADC_t, DR)     == 0x0040, "Wrong definition");
static_assert(offsetof(struct ADC_t, CCR)    == 0x0308, "Wrong definition");

#define ADC_ISR_AWD                        (   1 << 7)       // Analog watchdog flag
#define ADC_ISR_OVR                        (   1 << 4)       // ADC overrun
#define ADC_ISR_EOSEQ                      (   1 << 3)       // End of sequence flag
#define ADC_ISR_EOC                        (   1 << 2)       // End of conversion
#define ADC_ISR_EOSMP                      (   1 << 1)       // End of sampling flag
#define ADC_ISR_ADRDY                      (   1 << 0)       // ADC ready

#define ADC_IER_AWDIE                      (   1 << 7)       // Analog watchdog interrupt enable
#define ADC_IER_OVRIE                      (   1 << 4)       // Overrun interrupt enable
#define ADC_IER_EOSEQIE                    (   1 << 3)       // End of conversion sequence interrupt enable
#define ADC_IER_EOCIE                      (   1 << 2)       // End of conversion interrupt enable
#define ADC_IER_EOSMPIE                    (   1 << 1)       // End of sampling flag interrupt enable
#define ADC_IER_ADRDYIE                    (   1 << 0)       // ADC ready interrupt enable

#define ADC_CR_ADCAL                       (   1 << 31)      // ADC calibration
#define ADC_CR_ADSTP                       (   1 << 4)       // ADC stop conversion command
#define ADC_CR_ADSTART                     (   1 << 2)       // ADC start conversion command
#define ADC_CR_ADDIS                       (   1 << 1)       // ADC disable command
#define ADC_CR_ADEN                        (   1 << 0)       // ADC enable command

#define ADC_CFGR1_AWDCH                    (0x1F << 26)      // Analog watchdog channel selection
#define ADC_CFGR1_AWDEN                    (   1 << 23)      // Analog watchdog enabl
#define ADC_CFGR1_AWDSGL                   (   1 << 22)      // Enable the watchdog on a single channel or on all channels
#define ADC_CFGR1_DISCEN                   (   1 << 16)      // Discontinuous mode
#define ADC_CFGR1_AUTOFF                   (   1 << 15)      // Auto-off mode
#define ADC_CFGR1_WAIT                     (   1 << 14)      // Wait conversion mode
#define ADC_CFGR1_CONT                     (   1 << 13)      // Single / continuous conversion mode
#define ADC_CFGR1_OVRMOD                   (   1 << 12)      // Overrun management mode
#define ADC_CFGR1_EXTEN                    (0x03 << 10)      // External trigger enable and polarity selection
#define ADC_CFGR1_EXTSEL                   (0x07 << 6)       // External trigger selection
#define ADC_CFGR1_ALIGN                    (   1 << 5)       // Data alignment
#define ADC_CFGR1_RES                      (0x03 << 3)       // Data resolution
#define ADC_CFGR1_RES_12                   (0x00 << 3)       //   - 12 bits
#define ADC_CFGR1_RES_10                   (0x01 << 3)       //   - 10 bits
#define ADC_CFGR1_RES_8                    (0x02 << 3)       //   - 8 bits
#define ADC_CFGR1_RES_6                    (0x03 << 3)       //   - 6 bits
#define ADC_CFGR1_SCANDIR                  (   1 << 2)       // Scan sequence direction
#define ADC_CFGR1_DMACFG                   (   1 << 1)       // Direct memory access configuration
#define ADC_CFGR1_DMAEN                    (   1 << 0)       // Direct memory access enable

#define ADC_CFGR2_CKMODE                   (0x03 << 30)      // ADC clock mode
#define ADC_CFGR2_CKMODE_ADCCLK            (0x00 << 30)      //   - ADCCLK (Asynchronous clock mode)
#define ADC_CFGR2_CKMODE_PCLK2             (0x01 << 30)      //   - PCLK/2 (Synchronous clock mode)
#define ADC_CFGR2_CKMODE_PCLK4             (0x02 << 30)      //   - PCLK/4 (Synchronous clock mode)

#define ADC_SMPR_SMP                       (0x07 << 0)       // Sampling time selection
#define ADC_SMPR_SMP_0                     (0x00 << 0)       //   - 1.5 cycles
#define ADC_SMPR_SMP_7                     (0x01 << 0)       //   - 7.5 cycles
#define ADC_SMPR_SMP_13                    (0x02 << 0)       //   - 13.5 cycles
#define ADC_SMPR_SMP_28                    (0x03 << 0)       //   - 28.5 cycles
#define ADC_SMPR_SMP_41                    (0x04 << 0)       //   - 41.5 cycles
#define ADC_SMPR_SMP_55                    (0x05 << 0)       //   - 55.5 cycles
#define ADC_SMPR_SMP_71                    (0x06 << 0)       //   - 71.5 cycles
#define ADC_SMPR_SMP_239                   (0x07 << 0)       //   - 239.5 cycles

#define ADC_TR_HT                          (0x0FFF << 16)    // Analog watchdog higher threshold
#define ADC_TR_LT                          (0x0FFF << 0)     // Analog watchdog lower threshold

#define ADC_CHSELR_CHSEL17                 (   1 << 17)      // Channel 17 selection
#define ADC_CHSELR_CHSEL16                 (   1 << 16)      // Channel 16 selection
#define ADC_CHSELR_CHSEL15                 (   1 << 15)      // Channel 15 selection
#define ADC_CHSELR_CHSEL14                 (   1 << 14)      // Channel 14 selection
#define ADC_CHSELR_CHSEL13                 (   1 << 13)      // Channel 13 selection
#define ADC_CHSELR_CHSEL12                 (   1 << 12)      // Channel 12 selection
#define ADC_CHSELR_CHSEL11                 (   1 << 11)      // Channel 11 selection
#define ADC_CHSELR_CHSEL10                 (   1 << 10)      // Channel 10 selection
#define ADC_CHSELR_CHSEL9                  (   1 << 9)       // Channel 9 selection
#define ADC_CHSELR_CHSEL8                  (   1 << 8)       // Channel 8 selection
#define ADC_CHSELR_CHSEL7                  (   1 << 7)       // Channel 7 selection
#define ADC_CHSELR_CHSEL6                  (   1 << 6)       // Channel 6 selection
#define ADC_CHSELR_CHSEL5                  (   1 << 5)       // Channel 5 selection
#define ADC_CHSELR_CHSEL4                  (   1 << 4)       // Channel 4 selection
#define ADC_CHSELR_CHSEL3                  (   1 << 3)       // Channel 3 selection
#define ADC_CHSELR_CHSEL2                  (   1 << 2)       // Channel 2 selection
#define ADC_CHSELR_CHSEL1                  (   1 << 1)       // Channel 1 selection
#define ADC_CHSELR_CHSEL0                  (   1 << 0)       // Channel 0 selection

#define ADC_DR_DATA                        (0xFFFF << 0)     // Converted data

#define ADC_CCR_TSEN                       (   1 << 23)      // Temperature sensor enable
#define ADC_CCR_VREFEN                     (   1 << 22)      // Vrefint enable

// == TIM =============================================================================================================

struct TIM_t {
	_RW uint16_t CR1;            // Control register 1
	_RS uint16_t reserved1;
	_RW uint16_t CR2;            // Control register 2
	_RS uint16_t reserved2;
	_RW uint16_t SMCR;           // Slave mode control register
	_RS uint16_t reserved3;
	_RW uint16_t DIER;           // DMA/interrupt enable register
	_RS uint16_t reserved4;
	_RW uint16_t SR;             // Status register
	_RS uint16_t reserved5;
	_WO uint16_t EGR;            // Event generation register
	_RS uint16_t reserved6;
	_RW uint16_t CCMR1;          // Capture/compare mode register 1
	_RS uint16_t reserved7;
	_RW uint16_t CCMR2;          // Capture/compare mode register 2
	_RS uint16_t reserved8;
	_RW uint16_t CCER;           // Capture/compare enable register
	_RS uint16_t reserved9;
	_RW uint16_t CNT;            // Counter
	_RS uint16_t reserved10;
	_RW uint16_t PSC;            // Prescaler
	_RS uint16_t reserved11;
	_RW uint16_t ARR;            // Auto-reload register
	_RS uint16_t reserved12;
	_RW uint16_t RCR;            // Repetition counter register
	_RS uint16_t reserved13;
	_RW uint16_t CCR1;           // Capture/compare register 1
	_RS uint16_t reserved14;
	_RW uint16_t CCR2;           // Capture/compare register 2
	_RS uint16_t reserved15;
	_RW uint16_t CCR3;           // Capture/compare register 3
	_RS uint16_t reserved16;
	_RW uint16_t CCR4;           // Capture/compare register 4
	_RS uint16_t reserved17;
	_RW uint16_t BDTR;           // Break and dead-time register
	_RS uint16_t reserved18;
	_RW uint16_t DCR;            // DMA control register
	_RS uint16_t reserved19;
	_RW uint16_t DMAR;           // DMA address for full transfer
	_RS uint16_t reserved20;
};
static_assert(offsetof(struct TIM_t, DMAR) == 0x4C, "Wrong definition");

#define TIM_CR1_CKD                        (0x03 << 8)       // Clock division
#define TIM_CR1_ARPE                       (   1 << 7)       // Auto-reload preload enable
#define TIM_CR1_CMS                        (0x03 << 5)       // Center-aligned mode selection
#define TIM_CR1_DIR                        (   1 << 4)       // Direction
#define TIM_CR1_OPM                        (   1 << 3)       // One pulse mode
#define TIM_CR1_URS                        (   1 << 2)       // Update request source
#define TIM_CR1_UDIS                       (   1 << 1)       // Update disable
#define TIM_CR1_CEN                        (   1 << 0)       // Counter enable

#define TIM_CR2_OIS4                       (   1 << 14)      // Output idle state 4 (OC4 output)
#define TIM_CR2_OIS3N                      (   1 << 13)      // Output idle state 3 (OC3N output)
#define TIM_CR2_OIS3                       (   1 << 12)      // Output idle state 3 (OC3 output)
#define TIM_CR2_OIS2N                      (   1 << 11)      // Output idle state 2 (OC2N output)
#define TIM_CR2_OIS2                       (   1 << 10)      // Output idle state 2 (OC2 output)
#define TIM_CR2_OIS1N                      (   1 << 9)       // Output idle state 1 (OC1N output)
#define TIM_CR2_OIS1                       (   1 << 8)       // Output idle state 1 (OC1 output)
#define TIM_CR2_TI1S                       (   1 << 7)       // TI1 selection
#define TIM_CR2_MMS                        (0x07 << 4)       // Master mode selection
#define TIM_CR2_MMS_RESET                  (0x00 << 4)       //   - Reset
#define TIM_CR2_MMS_ENABLE                 (0x01 << 4)       //   - Enable
#define TIM_CR2_MMS_UPDATE                 (0x02 << 4)       //   - Update
#define TIM_CR2_MMS_CMPPULSE               (0x03 << 4)       //   - Compare pulse
#define TIM_CR2_MMS_CMP1                   (0x04 << 4)       //   - Compare - OC1REF
#define TIM_CR2_MMS_CMP2                   (0x05 << 4)       //   - Compare - OC2REF
#define TIM_CR2_MMS_CMP3                   (0x06 << 4)       //   - Compare - OC3REF
#define TIM_CR2_MMS_CMP4                   (0x07 << 4)       //   - Compare - OC4REF
#define TIM_CR2_CCDS                       (   1 << 3)       // Capture/compare DMA selection
#define TIM_CR2_CCUS                       (   1 << 2)       // Capture/compare control update selection
#define TIM_CR2_CCPC                       (   1 << 0)       // Capture/compare pre-loaded control

#define TIM_SMCR_ETP                       (   1 << 15)      // External trigger polarity
#define TIM_SMCR_ECE                       (   1 << 14)      // External clock enable
#define TIM_SMCR_ETPS                      (0x03 << 12)      // External trigger prescaler
#define TIM_SMCR_ETF                       (0x0F << 8)       // External trigger filter
#define TIM_SMCR_MSM                       (   1 << 7)       // Master/slave mode
#define TIM_SMCR_TS                        (0x07 << 4)       // Trigger selection
#define TIM_SMCR_TS_TRIG0                  (0x00 << 4)       //   - Internal trigger 0
#define TIM_SMCR_TS_TRIG1                  (0x01 << 4)       //   - Internal trigger 1
#define TIM_SMCR_TS_TRIG2                  (0x02 << 4)       //   - Internal trigger 2
#define TIM_SMCR_TS_TRIG3                  (0x03 << 4)       //   - Internal trigger 3
#define TIM_SMCR_TS_TI1                    (0x04 << 4)       //   - TI1 edge detector
#define TIM_SMCR_TS_TIM1                   (0x05 << 4)       //   - Filtered timer input 1
#define TIM_SMCR_TS_TIM2                   (0x06 << 4)       //   - Filtered timer input 2
#define TIM_SMCR_TS_EXT                    (0x07 << 4)       //   - External trigger input
#define TIM_SMCR_OCCS                      (   1 << 3)      // OCREF clear selection
#define TIM_SMCR_SMS                       (0x07 << 0)       // Slave mode selection
#define TIM_SMCR_SMS_DIS                   (0x00 << 0)       //   - Slave mode disabled
#define TIM_SMCR_SMS_ENC1                  (0x01 << 0)       //   - Encoder mode 1
#define TIM_SMCR_SMS_ENC2                  (0x02 << 0)       //   - Encoder mode 2
#define TIM_SMCR_SMS_ENC3                  (0x03 << 0)       //   - Encoder mode 3
#define TIM_SMCR_SMS_RESET                 (0x04 << 0)       //   - Reset mode
#define TIM_SMCR_SMS_GATED                 (0x05 << 0)       //   - Gated mode
#define TIM_SMCR_SMS_TRIG                  (0x06 << 0)       //   - Trigger mode
#define TIM_SMCR_SMS_EXT                   (0x07 << 0)       //   - External clock mode 1

#define TIM_DIER_TDE                       (   1 << 14)      // Trigger DMA request enable
#define TIM_DIER_COMDE                     (   1 << 13)      // COM DMA request enable
#define TIM_DIER_CC4DE                     (   1 << 12)      // Capture/compare 4 DMA request enable
#define TIM_DIER_CC3DE                     (   1 << 11)      // Capture/compare 3 DMA request enable
#define TIM_DIER_CC2DE                     (   1 << 10)      // Capture/compare 2 DMA request enable
#define TIM_DIER_CC1DE                     (   1 << 9)       // Capture/compare 1 DMA request enable
#define TIM_DIER_UDE                       (   1 << 8)       // Update DMA request enable
#define TIM_DIER_BIE                       (   1 << 7)       // Break interrupt enable
#define TIM_DIER_TIE                       (   1 << 6)       // Trigger interrupt enable
#define TIM_DIER_COMIE                     (   1 << 5)       // COM interrupt enable
#define TIM_DIER_CC4IE                     (   1 << 4)       // Capture/compare 4 interrupt enable
#define TIM_DIER_CC3IE                     (   1 << 3)       // Capture/compare 3 interrupt enable
#define TIM_DIER_CC2IE                     (   1 << 2)       // Capture/compare 2 interrupt enable
#define TIM_DIER_CC1IE                     (   1 << 1)       // Capture/compare 1 interrupt enable
#define TIM_DIER_UIE                       (   1 << 0)       // Update interrupt enable

#define TIM_SR_CC4OF                       (   1 << 12)      // Capture/compare 4 overcapture flag
#define TIM_SR_CC3OF                       (   1 << 11)      // Capture/compare 3 overcapture flag
#define TIM_SR_CC2OF                       (   1 << 10)      // Capture/compare 2 overcapture flag
#define TIM_SR_CC1OF                       (   1 << 9)       // Capture/compare 1 overcapture flag
#define TIM_SR_BIF                         (   1 << 7)       // Break interrupt flag
#define TIM_SR_TIF                         (   1 << 6)       // Trigger interrupt flag
#define TIM_SR_COMIF                       (   1 << 5)       // COM interrupt flag
#define TIM_SR_CC4IF                       (   1 << 4)       // Capture/compare 4 interrupt flag
#define TIM_SR_CC3IF                       (   1 << 3)       // Capture/compare 3 interrupt flag
#define TIM_SR_CC2IF                       (   1 << 2)       // Capture/compare 2 interrupt flag
#define TIM_SR_CC1IF                       (   1 << 1)       // Capture/compare 1 interrupt flag
#define TIM_SR_UIF                         (   1 << 0)       // Update interrupt flag

#define TIM_EGR_BG                         (   1 << 7)       // Break generation
#define TIM_EGR_TG                         (   1 << 6)       // Trigger generation
#define TIM_EGR_COMG                       (   1 << 5)       // Capture/compare control update generation
#define TIM_EGR_CC4G                       (   1 << 4)       // Capture/compare 4 generation
#define TIM_EGR_CC3G                       (   1 << 3)       // Capture/compare 3 generation
#define TIM_EGR_CC2G                       (   1 << 2)       // Capture/compare 2 generation
#define TIM_EGR_CC1G                       (   1 << 1)       // Capture/compare 1 generation
#define TIM_EGR_UG                         (   1 << 0)       // Update generation

#define TIM_CCMR1_OC2CE                    (   1 << 15)      // Output compare 2 clear enable
#define TIM_CCMR1_OC2M                     (0x07 << 12)      // Output compare 2 mode
#define TIM_CCMR1_OC2M_FROZEN              (0x00 << 12)      //   - Frozen
#define TIM_CCMR1_OC2M_ACTIVE              (0x01 << 12)      //   - Set channel 2 to active level on match
#define TIM_CCMR1_OC2M_INACTIVE            (0x02 << 12)      //   - Set channel 2 to inactive level on match
#define TIM_CCMR1_OC2M_TOGGLE              (0x03 << 12)      //   - Toggle
#define TIM_CCMR1_OC2M_FORCELOW            (0x04 << 12)      //   - Force inactive level
#define TIM_CCMR1_OC2M_FORCEHIGH           (0x05 << 12)      //   - Force active level
#define TIM_CCMR1_OC2M_PWM1                (0x06 << 12)      //   - PWM mode 1
#define TIM_CCMR1_OC2M_PWM2                (0x07 << 12)      //   - PWM mode 2
#define TIM_CCMR1_OC2PE                    (   1 << 11)      // Output compare 2 preload enable
#define TIM_CCMR1_OC2FE                    (   1 << 10)      // Output compare 2 fast enable
#define TIM_CCMR1_IC2F                     (0x0F << 12)      // Input capture 2 filter
#define TIM_CCMR1_IC2PSC                   (0x03 << 10)      // Input capture 2 prescaler
#define TIM_CCMR1_CC2S                     (0x03 << 8)       // Capture/compare 2 selection
#define TIM_CCMR1_CC2S_OUT                 (0x00 << 8)       //   - CC2 channel is configured as output
#define TIM_CCMR1_CC2S_TI2                 (0x01 << 8)       //   - CC2 channel is configured as input, IC2 is mapped on TI2
#define TIM_CCMR1_CC2S_TI1                 (0x02 << 8)       //   - CC2 channel is configured as input, IC2 is mapped on TI1
#define TIM_CCMR1_CC2S_TRC                 (0x03 << 8)       //   - CC2 channel is configured as input, IC2 is mapped on TRC
#define TIM_CCMR1_OC1CE                    (   1 << 7)       // Output compare 1 clear enable
#define TIM_CCMR1_OC1M                     (0x07 << 4)       // Output compare 1 mode
#define TIM_CCMR1_OC1M_FROZEN              (0x00 << 4)       //   - Frozen
#define TIM_CCMR1_OC1M_ACTIVE              (0x01 << 4)       //   - Set channel 1 to active level on match
#define TIM_CCMR1_OC1M_INACTIVE            (0x02 << 4)       //   - Set channel 1 to inactive level on match
#define TIM_CCMR1_OC1M_TOGGLE              (0x03 << 4)       //   - Toggle
#define TIM_CCMR1_OC1M_FORCELOW            (0x04 << 4)       //   - Force inactive level
#define TIM_CCMR1_OC1M_FORCEHIGH           (0x05 << 4)       //   - Force active level
#define TIM_CCMR1_OC1M_PWM1                (0x06 << 4)       //   - PWM mode 1
#define TIM_CCMR1_OC1M_PWM2                (0x07 << 4)       //   - PWM mode 2
#define TIM_CCMR1_IC1F                     (0x0F << 4)       // Input capture 1 filter
#define TIM_CCMR1_OC1PE                    (   1 << 3)       // Output compare 1 preload enable
#define TIM_CCMR1_OC1FE                    (   1 << 2)       // Output compare 1 fast enable
#define TIM_CCMR1_IC1PSC                   (0x03 << 2)       // Input capture 1 prescaler
#define TIM_CCMR1_CC1S                     (0x03 << 0)       // Capture/compare 1 selection
#define TIM_CCMR1_CC1S_OUT                 (0x00 << 0)       //   - CC1 channel is configured as output
#define TIM_CCMR1_CC1S_TI1                 (0x01 << 0)       //   - CC1 channel is configured as input, IC1 is mapped on TI1
#define TIM_CCMR1_CC1S_TI2                 (0x02 << 0)       //   - CC1 channel is configured as input, IC1 is mapped on TI2
#define TIM_CCMR1_CC1S_TRC                 (0x03 << 0)       //   - CC1 channel is configured as input, IC1 is mapped on TRC

#define TIM_CCMR2_OC4CE                    (   1 << 15)      // Output compare 4 clear enable
#define TIM_CCMR2_OC4M                     (0x07 << 12)      // Output compare 4 mode
#define TIM_CCMR2_OC4M_FROZEN              (0x00 << 12)      //   - Frozen
#define TIM_CCMR2_OC4M_ACTIVE              (0x01 << 12)      //   - Set channel 4 to active level on match
#define TIM_CCMR2_OC4M_INACTIVE            (0x02 << 12)      //   - Set channel 4 to inactive level on match
#define TIM_CCMR2_OC4M_TOGGLE              (0x03 << 12)      //   - Toggle
#define TIM_CCMR2_OC4M_FORCELOW            (0x04 << 12)      //   - Force inactive level
#define TIM_CCMR2_OC4M_FORCEHIGH           (0x05 << 12)      //   - Force active level
#define TIM_CCMR2_OC4M_PWM1                (0x06 << 12)      //   - PWM mode 1
#define TIM_CCMR2_OC4M_PWM2                (0x07 << 12)      //   - PWM mode 2
#define TIM_CCMR2_OC4PE                    (   1 << 11)      // Output compare 4 preload enable
#define TIM_CCMR2_OC4FE                    (   1 << 10)      // Output compare 4 fast enable
#define TIM_CCMR2_IC4F                     (0x0F << 12)      // Input capture 4 filter
#define TIM_CCMR2_IC4PSC                   (0x03 << 10)      // Input capture 4 prescaler
#define TIM_CCMR2_CC4S                     (0x03 << 8)       // Capture/compare 4 selection
#define TIM_CCMR2_CC4S_OUT                 (0x00 << 8)       //   - CC4 channel is configured as output
#define TIM_CCMR2_CC4S_TI4                 (0x01 << 8)       //   - CC4 channel is configured as input, IC4 is mapped on TI4
#define TIM_CCMR2_CC4S_TI3                 (0x02 << 8)       //   - CC4 channel is configured as input, IC4 is mapped on TI3
#define TIM_CCMR2_CC4S_TRC                 (0x03 << 8)       //   - CC4 channel is configured as input, IC4 is mapped on TRC
#define TIM_CCMR2_OC3CE                    (   1 << 7)       // Output compare 3 clear enable
#define TIM_CCMR2_OC3M                     (0x07 << 4)       // Output compare 3 mode
#define TIM_CCMR2_OC3M_FROZEN              (0x00 << 4)       //   - Frozen
#define TIM_CCMR2_OC3M_ACTIVE              (0x01 << 4)       //   - Set channel 3 to active level on match
#define TIM_CCMR2_OC3M_INACTIVE            (0x02 << 4)       //   - Set channel 3 to inactive level on match
#define TIM_CCMR2_OC3M_TOGGLE              (0x03 << 4)       //   - Toggle
#define TIM_CCMR2_OC3M_FORCELOW            (0x04 << 4)       //   - Force inactive level
#define TIM_CCMR2_OC3M_FORCEHIGH           (0x05 << 4)       //   - Force active level
#define TIM_CCMR2_OC3M_PWM1                (0x06 << 4)       //   - PWM mode 1
#define TIM_CCMR2_OC3M_PWM2                (0x07 << 4)       //   - PWM mode 2
#define TIM_CCMR2_IC3F                     (0x0F << 4)       // Input capture 1 filter
#define TIM_CCMR2_OC3PE                    (   1 << 3)       // Output compare 1 preload enable
#define TIM_CCMR2_OC3FE                    (   1 << 2)       // Output compare 1 fast enable
#define TIM_CCMR2_IC3PSC                   (0x03 << 2)       // Input capture 1 prescaler
#define TIM_CCMR2_CC3S                     (0x03 << 0)       // Capture/compare 1 selection
#define TIM_CCMR2_CC3S_OUT                 (0x00 << 0)       //   - CC1 channel is configured as output
#define TIM_CCMR2_CC3S_TI3                 (0x01 << 0)       //   - CC1 channel is configured as input, IC1 is mapped on TI3
#define TIM_CCMR2_CC3S_TI4                 (0x02 << 0)       //   - CC1 channel is configured as input, IC1 is mapped on TI4
#define TIM_CCMR2_CC3S_TRC                 (0x03 << 0)       //   - CC1 channel is configured as input, IC1 is mapped on TRC

#define TIM_CCER_CC4P                      (   1 << 13)      // Capture/compare 4 output polarity
#define TIM_CCER_CC4E                      (   1 << 12)      // Capture/compare 4 output enable
#define TIM_CCER_CC3NP                     (   1 << 11)      // Capture/compare 3 complementary output polarity
#define TIM_CCER_CC3NE                     (   1 << 10)      // Capture/compare 3 complementary output enable
#define TIM_CCER_CC3P                      (   1 << 9)       // Capture/compare 3 output polarity
#define TIM_CCER_CC3E                      (   1 << 8)       // Capture/compare 3 output enable
#define TIM_CCER_CC2NP                     (   1 << 7)       // Capture/compare 2 complementary output polarity
#define TIM_CCER_CC2NE                     (   1 << 6)       // Capture/compare 2 complementary output enable
#define TIM_CCER_CC2P                      (   1 << 5)       // Capture/compare 2 output polarity
#define TIM_CCER_CC2E                      (   1 << 4)       // Capture/compare 2 output enable
#define TIM_CCER_CC1NP                     (   1 << 3)       // Capture/compare 1 complementary output polarity
#define TIM_CCER_CC1NE                     (   1 << 2)       // Capture/compare 1 complementary output enable
#define TIM_CCER_CC1P                      (   1 << 1)       // Capture/compare 1 output polarity
#define TIM_CCER_CC1E                      (   1 << 0)       // Capture/compare 1 output enable

#define TIM_CNT_CNT                        (0xFFFF << 0)     // Counter value

#define TIM_PSC_PSC                        (0xFFFF << 0)     // Prescaler value

#define TIM_ARR_ARR                        (0xFFFF << 0)     // Auto-reload value

#define TIM_RCR_REP                        (0xFF << 0)       // Repetition counter value

#define TIM_CCR1_CCR1                      (0xFFFF << 0)     // Capture/compare 1 value

#define TIM_CCR2_CCR2                      (0xFFFF << 0)     // Capture/compare 2 value

#define TIM_CCR3_CCR3                      (0xFFFF << 0)     // Capture/compare 3 value

#define TIM_CCR4_CCR4                      (0xFFFF << 0)     // Capture/compare 4 value

#define TIM_BDTR_MOE                       (   1 << 15)      // Main output enable
#define TIM_BDTR_AOE                       (   1 << 14)      // Automatic output enable
#define TIM_BDTR_BKP                       (   1 << 13)      // Break polarity
#define TIM_BDTR_BKE                       (   1 << 12)      // Break enable
#define TIM_BDTR_OSSR                      (   1 << 11)      // Off-state selection for Run mode
#define TIM_BDTR_OSSI                      (   1 << 10)      // Off-state selection for Idle mode
#define TIM_BDTR_LOCK                      (0x03 << 8)       // Lock configuration
#define TIM_BDTR_LOCK_OFF                  (0x00 << 8)       //   - Lock off
#define TIM_BDTR_LOCK_LEVEL1               (0x01 << 8)       //   - Lock level 1
#define TIM_BDTR_LOCK_LEVEL2               (0x02 << 8)       //   - Lock level 2
#define TIM_BDTR_LOCK_LEVEL3               (0x03 << 8)       //   - Lock level 3
#define TIM_BDTR_DTG                       (0xFF << 0)       // Dead-time generator setup

#define TIM_DCR_DBL                        (0x1F << 8)       // DMA burst length
#define TIM_DCR_DBA                        (0x1F << 0)       // DMA base address

#define TIM_DMAR_DMAB                      (0xFFFF << 0)     // DMA register for burst accesses

// == RTC =============================================================================================================

struct RTC_t {
	_RW uint32_t TR;             // Time register
	_RW uint32_t DR;             // Date register
	_RW uint32_t CR;             // Control register
	_RW uint32_t ISR;            // Initialization and status register
	_RW uint32_t PRER;           // Prescaler register
	_RW uint32_t WUTR;           // Wakeup timer register
	_RS uint32_t reserved1;
	_RW uint32_t ALRMAR;         // Alarm A register
	_RS uint32_t reserved2;
	_RW uint32_t WPR;            // Write protection register
	_RO uint32_t SSR;            // Sub second register
	_RW uint32_t SHIFTR;         // Shift control register
	_RO uint32_t TSTR;           // Timestamp time register
	_RO uint32_t TSDR;           // Timestamp date register
	_RO uint32_t TSSSR;          // Timestamp sub second register
	_RW uint32_t CAR;            // Calibration register
	_RW uint32_t TAFCR;          // Tamper and alternate function configuration register
	_RW uint32_t ALRMASSR;       // RTC alarm A sub second register
};
static_assert(offsetof(struct RTC_t, ALRMASSR) == 0x44, "Wrong definition");

#define RTC_TR_PM                          (   1 << 22)      // AM/PM notation
#define RTC_TR_HT                          (0x03 << 20)      // Hour tens in BCD format
#define RTC_TR_HU                          (0x0F << 16)      // Hour units in BCD format
#define RTC_TR_MNT                         (0x07 << 12)      // Minute tens in BCD format
#define RTC_TR_MNU                         (0x0F << 8)       // Minute units in BCD format
#define RTC_TR_ST                          (0x07 << 4)       // Second tens in BCD format
#define RTC_TR_SU                          (0x0F << 0)       // Second units in BCD format

#define RTC_DR_YT                          (0x0F << 20)      // Year tens in BCD format
#define RTC_DR_YU                          (0x0F << 16)      // Year units in BCD format
#define RTC_DR_WDU                         (0x07 << 13)      // Week day units
#define RTC_DR_MT                          (   1 << 12)      // Month tens in BCD format
#define RTC_DR_MU                          (0x0F << 8)       // Month units in BCD format
#define RTC_DR_DT                          (0x03 << 4)       // Date tens in BCD format
#define RTC_DR_DU                          (0x0F << 0)       // Date units in BCD format

#define RTC_CR_COE                         (   1 << 23)      // Calibration output enable
#define RTC_CR_OSEL                        (0x03 << 21)      // Output selection
#define RTC_CR_POL                         (   1 << 20)      // Output polarity
#define RTC_CR_COSEL                       (   1 << 19)      // Calibration output selection
#define RTC_CR_BKP                         (   1 << 18)      // Backup
#define RTC_CR_SUB1H                       (   1 << 17)      // Subtract 1 hour (winter time change)
#define RTC_CR_ADD1H                       (   1 << 16)      // Add 1 hour (summer time change)
#define RTC_CR_TSIE                        (   1 << 15)      // Timestamp interrupt enable
#define RTC_CR_WUTIE                       (   1 << 14)      // Wakeup timer interrupt enable
#define RTC_CR_ALRAIE                      (   1 << 12)      // Alarm A interrupt enable
#define RTC_CR_TSE                         (   1 << 11)      // Timestamp enable
#define RTC_CR_WUTE                        (   1 << 10)      // Wakeup timer enable
#define RTC_CR_ALRAE                       (   1 << 8)       // Alarm A enable
#define RTC_CR_FMT                         (   1 << 6)       // Hour format
#define RTC_CR_BYPSHAD                     (   1 << 5)       // Bypass the shadow registers
#define RTC_CR_REFCKON                     (   1 << 4)       // RTC_REFIN reference clock detection enable (50 or 60 Hz)
#define RTC_CR_TSEDGE                      (   1 << 3)       // Time-stamp event active edge
#define RTC_CR_WUCKSEL                     (0x07 << 0)       // Wakeup clock selection

#define RTC_ISR_RECALPF                    (   1 << 16)      // Recalibration pending flag
#define RTC_ISR_TAMP2F                     (   1 << 14)      // RTC_TAMP2 detection flag
#define RTC_ISR_TAMP1F                     (   1 << 13)      // RTC_TAMP1 detection flag
#define RTC_ISR_TSOVF                      (   1 << 12)      // Timestamp overflow flag
#define RTC_ISR_TSF                        (   1 << 11)      // Timestamp flag
#define RTC_ISR_WUTF                       (   1 << 10)      // Wakeup timer flag
#define RTC_ISR_ALRAF                      (   1 << 8)       // Alarm A flag
#define RTC_ISR_INIT                       (   1 << 7)       // Initialization mode
#define RTC_ISR_INITF                      (   1 << 6)       // Initialization flag
#define RTC_ISR_RSF                        (   1 << 5)       // Registers synchronization flag
#define RTC_ISR_INITS                      (   1 << 4)       // Initialization status flag
#define RTC_ISR_SHPF                       (   1 << 3)       // Shift operation pending
#define RTC_ISR_WUTWF                      (   1 << 2)       // Wakeup timer write flag
#define RTC_ISR_ALRAWF                     (   1 << 0)       // Alarm A write flag

#define RTC_PRER_PREDIVA                   (0x07 << 16)      // Asynchronous prescaler factor
#define RTC_PRER_PREDIVS                   (0x7FFF << 0)     // Synchronous prescaler factor

#define RTC_WUTR_WUT                       (0xFFFF << 0)     // Wakeup auto-reload value bits

#define RTC_ALRMAR_MSK4                    (   1 << 31)      // Alarm A date mask
#define RTC_ALRMAR_WDSEL                   (   1 << 30)      // Week day selection
#define RTC_ALRMAR_DT                      (0x03 << 28)      // Date tens in BCD format
#define RTC_ALRMAR_DU                      (0x0F << 24)      // Date units or day in BCD format
#define RTC_ALRMAR_MSK3                    (   1 << 23)      // Alarm A hours mask
#define RTC_ALRMAR_PM                      (   1 << 22)      // AM/PM notation
#define RTC_ALRMAR_HT                      (0x03 << 20)      // Hour tens in BCD format
#define RTC_ALRMAR_HU                      (0x0F << 16)      // Hour units in BCD format
#define RTC_ALRMAR_MSK2                    (   1 << 15)      // Alarm A minutes mask
#define RTC_ALRMAR_MNT                     (0x07 << 12)      // Minute tens in BCD format
#define RTC_ALRMAR_MNU                     (0x0F << 8)       // Minute units in BCD format
#define RTC_ALRMAR_MSK1                    (   1 << 7)       // Alarm A seconds mask
#define RTC_ALRMAR_ST                      (0x07 << 4)       // Second tens in BCD format
#define RTC_ALRMAR_SU                      (0x0F << 0)       // Second units in BCD format

#define RTC_WPR_KEY                        (0xFF << 0)       // Write protection key

#define RTC_SSR_SS                         (0xFFFF << 0)     // Sub second value

#define RTC_SHIFTR_ADD1S                   (   1 << 31)      // Add one second
#define RTC_SHIFTR_SUBFS                   (0x7FFF << 0)     // Subtract a fraction of a second

#define RTC_TSTR_PM                        (   1 << 22)      // AM/PM notation
#define RTC_TSTR_HT                        (0x03 << 20)      // Hour tens in BCD format
#define RTC_TSTR_HU                        (0x0F << 16)      // Hour units in BCD format
#define RTC_TSTR_MNT                       (0x07 << 12)      // Minute tens in BCD format
#define RTC_TSTR_MNU                       (0x0F << 8)       // Minute units in BCD format
#define RTC_TSTR_ST                        (0x07 << 4)       // Second tens in BCD format
#define RTC_TSTR_SU                        (0x0F << 0)       // Second units in BCD format

#define RTC_TSDR_WDU                       (0x07 << 13)      // Week day units
#define RTC_TSDR_MT                        (   1 << 12)      // Month tens in BCD format
#define RTC_TSDR_MU                        (0x0F << 8)       // Month units in BCD format
#define RTC_TSDR_DT                        (0x03 << 4)       // Date tens in BCD format
#define RTC_TSDR_DU                        (0x0F << 0)       // Date units in BCD format

#define RTC_TSSSR_SS                       (0xFFFF << 0)     // Sub second value

#define RTC_CALR_CALP                      (   1 << 15)      // Increase frequency of RTC by 488.5 ppm
#define RTC_CALR_CALW8                     (   1 << 14)      // Use an 8-second calibration cycle period
#define RTC_CALR_CALW16                    (   1 << 13)      // Use a 16-second calibration cycle period
#define RTC_CALR_CALM                      (0xFF << 0);       // Calibration minus

#define RTC_TAFCR_PC15MODE                 (   1 << 23)      // PC15 mode
#define RTC_TAFCR_PC15VALUE                (   1 << 22)      // PC15 value
#define RTC_TAFCR_PC14MODE                 (   1 << 21)      // PC14 mode
#define RTC_TAFCR_PC14VALUE                (   1 << 20)      // PC14 value
#define RTC_TAFCR_PC13MODE                 (   1 << 19)      // PC13 mode
#define RTC_TAFCR_PC13VALUE                (   1 << 18)      // RTC_ALARM output type/PC13 value
#define RTC_TAFCR_TAMPPUDIS                (   1 << 15)      // RTC_TAMPx pull-up disable
#define RTC_TAFCR_TAMPPRCH                 (   1 << 13)      // RTC_TAMPx precharge duration
#define RTC_TAFCR_TAMPFLT                  (   1 << 11)      // RTC_TAMPx filter count
#define RTC_TAFCR_TAMPFREQ                 (   1 << 8)       // Tamper sampling frequency
#define RTC_TAFCR_TAMPTS                   (   1 << 7)       // Activate timestamp on tamper detection event
#define RTC_TAFCR_TAMP2TRG                 (   1 << 4)       // Active level for RTC_TAMP2 input
#define RTC_TAFCR_TAMP2E                   (   1 << 3)       // RTC_TAMP2 input detection enable
#define RTC_TAFCR_TAMPIE                   (   1 << 2)       // Tamper interrupt enable
#define RTC_TAFCR_TAMP1TRG                 (   1 << 1)       // Active level for RTC_TAMP1 input
#define RTC_TAFCR_TAMP1E                   (   1 << 0)       // RTC_TAMP1 input detection enable

#define RTC_ALRMASSR_MASKSS                (0x0F << 24)      // Mask the most-significant bits starting at this bit
#define RTC_ALRMASSR_SS                    (0x7FFF << 0)     // Sub seconds value

// == IWDG ============================================================================================================

struct IWDG_t {
	_WO uint32_t KR;             // Key register
	_RW uint32_t PR;             // Prescaler register
	_RW uint32_t RLR;            // Reload register
	_RO uint32_t SR;             // Status register
	_RW uint32_t WINR;           // Window register
};
static_assert(offsetof(struct IWDG_t, WINR) == 0x10, "Wrong definition");

#define IWDG_KR_KEY                        (0xFFFF << 0)     // Key value

#define IWDG_PR_PR                         (0x07 << 0)       // Prescaler divider

#define IWDG_RLR_RL                        (0x0FFF << 0)     // Watchdog counter reload value

#define IWDG_SR_WVU                        (   1 << 2)       // Watchdog counter window value update
#define IWDG_SR_RVU                        (   1 << 1)       // Watchdog counter reload value update
#define IWDG_SR_PVU                        (   1 << 0)       // Watchdog prescaler value update

#define IWDG_WINR_WIN                      (0x0FFF << 0)     // Watchdog counter window value

// == WWDG ============================================================================================================

struct WWDG_t {
	_RW uint32_t CR;             // Control register
	_RW uint32_t CFR;            // Configuration register
	_RW uint32_t SR;             // Status register
};
static_assert(offsetof(struct WWDG_t, SR) == 0x08, "Wrong definition");

#define WWDG_CR_WDGA                       (   1 << 7)       // Activation bit
#define WWDG_CR_T                          (0x7F << 0)       // 7-bit counter (MSB to LSB)

#define WWDG_CFR_EWI                       (   1 << 9)       // Early wakeup interrupt
#define WWDG_CFR_WDGTB                     (0x03 << 7)       // Timer base
#define WWDG_CFR_W                         (0x7F << 0)       // 7-bit window value

#define WWDG_SR_EWIF                       (   1 << 0)       // Early wakeup interrupt flag

// == USB =============================================================================================================

typedef uint16_t UsbBufAddr_t;

struct USB_t {
	_RW uint16_t EP0R;           // Endpoint 0 register
	_RS uint16_t reserved1;
	_RW uint16_t EP1R;           // Endpoint 1 register
	_RS uint16_t reserved2;
	_RW uint16_t EP2R;           // Endpoint 2 register
	_RS uint16_t reserved3;
	_RW uint16_t EP3R;           // Endpoint 3 register
	_RS uint16_t reserved4;
	_RW uint16_t EP4R;           // Endpoint 4 register
	_RS uint16_t reserved5;
	_RW uint16_t EP5R;           // Endpoint 5 register
	_RS uint16_t reserved6;
	_RW uint16_t EP6R;           // Endpoint 6 register
	_RS uint16_t reserved7;
	_RW uint16_t EP7R;           // Endpoint 7 register
	_RS uint16_t reserved8;
	_RS uint16_t reserved9[16];
	_RW uint16_t CNTR;           // Control register
	_RS uint16_t reserved10;
	_RW uint16_t ISTR;           // Interrupt status register
	_RS uint16_t reserved11;
	_RO uint16_t FNR;            // Frame number register
	_RS uint16_t reserved12;
	_RW uint16_t DADDR;          // Device address
	_RS uint16_t reserved13;
	_RW UsbBufAddr_t BTABLE;     // Buffer table address
	_RS uint16_t reserved14;
	_RW uint16_t LPMCSR;         // LPM control and status register
	_RS uint16_t reserved15;
	_RW uint16_t BCDR;           // Battery charging detector
};
static_assert(offsetof(struct USB_t, CNTR) == 0x40, "Wrong definition");
static_assert(offsetof(struct USB_t, BCDR) == 0x58, "Wrong definition");

struct UsbBufSlot_t {
	_RW UsbBufAddr_t ADDR;       // Buffer address
	_RS uint16_t reserved1;
	_RW uint16_t COUNT;          // Byte count
	_RS uint16_t reserved2;
};

union UsbBuf_t {
	struct {                     // Bi-direction EP:
		struct UsbBufSlot_t tx;           //   Transmission
		struct UsbBufSlot_t rx;           //   Reception
	} bi;
	struct {                     // Double-buffered EP:
		struct UsbBufSlot_t _0;           //   Data 0
		struct UsbBufSlot_t _1;           //   Data 1
	} db;
};
static_assert(sizeof(union UsbBuf_t) == 16, "Wrong definition");

#define USB_EPR_CTRRX                      (   1 << 15)      // Correct transfer for reception
#define USB_EPR_DTOGRX                     (   1 << 14)      // Data toggle, for reception transfers
#define USB_EPR_SWBUFTX                    (   1 << 14)      // Which buffer is used by application, for IN double-buffered EP only
#define USB_EPR_STATRX                     (0x03 << 12)      // Status bits, for reception transfers
#define USB_EPR_STATRX_DISABLED            (0x00 << 12)      //   - all reception requests addressed to this endpoint are ignored
#define USB_EPR_STATRX_STALL               (0x01 << 12)      //   - the endpoint is stalled and all reception requests result in a STALL handshake
#define USB_EPR_STATRX_NAK                 (0x02 << 12)      //   - the endpoint is naked and all reception requests result in a NAK handshake
#define USB_EPR_STATRX_VALID               (0x03 << 12)      //   - this endpoint is enabled for reception
#define USB_EPR_SETUP                      (   1 << 11)      // Setup transaction completed
#define USB_EPR_EPTYPE                     (0x03 << 9)       // Endpoint type
#define USB_EPR_EPTYPE_BULK                (0x00 << 9)       //   - Bulk
#define USB_EPR_EPTYPE_CONTROL             (0x01 << 9)       //   - Control
#define USB_EPR_EPTYPE_ISO                 (0x02 << 9)       //   - Isochronous
#define USB_EPR_EPTYPE_INTERRUPT           (0x03 << 9)       //   - Interrupt
#define USB_EPR_EPKIND                     (   1 << 8)       // Endpoint kind
#define USB_EPR_DBLBUF                     (   1 << 8)       // Double buffer enabled, only for bulk DP
#define USB_EPR_STATUSOUT                  (   1 << 8)       // Status OUT is expected, only for control DP
#define USB_EPR_CTRTX                      (   1 << 7)       // Correct transfer for transmission
#define USB_EPR_DTOGTX                     (   1 << 6)       // Data toggle, for transmission transfers
#define USB_EPR_SWBUFRX                    (   1 << 6)       // Which buffer is used by application, for OUT double-buffered EP only
#define USB_EPR_STATTX                     (0x03 << 4)       // Status bits, for transmission transfers
#define USB_EPR_STATTX_DISABLED            (0x00 << 4)       //   - all transmission requests addressed to this endpoint are ignored
#define USB_EPR_STATTX_STALL               (0x01 << 4)       //   - the endpoint is stalled and all transmission requests result in a STALL handshake
#define USB_EPR_STATTX_NAK                 (0x02 << 4)       //   - the endpoint is naked and all transmission requests result in a NAK handshake
#define USB_EPR_STATTX_VALID               (0x03 << 4)       //   - this endpoint is enabled for transmission
#define USB_EPR_EA                         (0x0F << 0)       // Endpoint address

#define USB_CNTR_CTRM                      (   1 << 15)      // Correct transfer interrupt mask
#define USB_CNTR_PMAOVRM                   (   1 << 14)      // Packet memory area over / underrun interrupt mask
#define USB_CNTR_ERRM                      (   1 << 13)      // Error interrupt mask
#define USB_CNTR_WKUPM                     (   1 << 12)      // Wakeup interrupt mask
#define USB_CNTR_SUSPM                     (   1 << 11)      // Suspend mode interrupt mask
#define USB_CNTR_RESETM                    (   1 << 10)      // USB reset interrupt mask
#define USB_CNTR_SOFM                      (   1 << 9)       // Start of frame interrupt mask
#define USB_CNTR_ESOFM                     (   1 << 8)       // Expected start of frame interrupt mask
#define USB_CNTR_L1REQM                    (   1 << 7)       // LPM L1 state request interrupt mask
#define USB_CNTR_L1RESUME                  (   1 << 5)       // LPM L1 Resume request
#define USB_CNTR_RESUME                    (   1 << 4)       // Resume request
#define USB_CNTR_FSUSP                     (   1 << 3)       // Force suspend
#define USB_CNTR_LPMODE                    (   1 << 2)       // Low-power mode
#define USB_CNTR_PDWN                      (   1 << 1)       // Power down
#define USB_CNTR_FRES                      (   1 << 0)       // Force USB reset

#define USB_ISTR_CTR                       (   1 << 15)      // Correct transfer
#define USB_ISTR_PMAOVR                    (   1 << 14)      // Packet memory area over / underrun
#define USB_ISTR_ERR                       (   1 << 13)      // Error
#define USB_ISTR_WKUP                      (   1 << 12)      // Wakeup
#define USB_ISTR_SUSP                      (   1 << 11)      // Suspend mode request
#define USB_ISTR_RESET                     (   1 << 10)      // USB reset request
#define USB_ISTR_SOF                       (   1 << 9)       // Start of frame
#define USB_ISTR_ESOF                      (   1 << 8)       // Expected start of frame
#define USB_ISTR_L1REQ                     (   1 << 7)       // LPM L1 state request
#define USB_ISTR_DIR                       (   1 << 4)       // Direction of transaction
#define USB_ISTR_EPID                      (0x0F << 0)       // Endpoint identifier

#define USB_FRN_RXDP                       (   1 << 15)      // Receive data + line status
#define USB_FRN_RXDM                       (   1 << 14)      // Receive data - line status
#define USB_FRN_LCK                        (   1 << 13)      // Locked
#define USB_FRN_LSOF                       (0x03 << 11)      // Lost SOF
#define USB_FRN_FN                         (0x07FF << 0)     // Frame number

#define USB_DADDR_EF                       (   1 << 7)       // Enable function
#define USB_DADDR_ADD                      (0x7F << 0)       // Device address

#define USB_LPMCSR_BESL                    (0x0F << 4)       // BESL value
#define USB_LPMCSR_REMWAKE                 (   1 << 3)       // bRemoteWake value
#define USB_LPMCSR_LPMACK                  (   1 << 1)       // LPM Token acknowledge enable
#define USB_LPMCSR_LPMEN                   (   1 << 0)       // LPM support enable

#define USB_BCDR_DPPU                      (   1 << 15)      // DP pull-up control
#define USB_BCDR_PS2DET                    (   1 << 7)       // DM pull-up detection status
#define USB_BCDR_SDET                      (   1 << 6)       // Secondary detection (SD) status
#define USB_BCDR_PDET                      (   1 << 5)       // Primary detection (PD) status
#define USB_BCDR_DCDET                     (   1 << 4)       // Data contact detection (DCD) status
#define USB_BCDR_SDEN                      (   1 << 3)       // Secondary detection (SD) mode enable
#define USB_BCDR_PDEN                      (   1 << 2)       // Primary detection (PD) mode enable
#define USB_BCDR_DCDEN                     (   1 << 1)       // Data contact detection (DCD) mode enable
#define USB_BCDR_BCDEN                     (   1 << 0)       // Battery charging detector (BCD) enable

#define USBBUF_COUNTTX_COUNTTX             (0x03FF << 0)     // Transmission byte count

#define USBBUF_COUNTRX_BLSIZE              (   1 << 15)      // BLock size
#define USBBUF_COUNTRX_NUMBLOCK            (0x1F << 10)      // Number of blocks
#define USBBUF_COUNTRX_COUNTRX             (0x03FF << 0)     // Reception byte count

// == SPI =============================================================================================================

struct SPI_t {
	_RW uint16_t CR1;            // Control register 1
	_RS uint16_t reserved1;
	_RW uint16_t CR2;            // Control register 2
	_RS uint16_t reserved2;
	_RO uint16_t SR;             // Status register
	_RS uint16_t reserved3;
	_RW uint16_t DR;             // Data register
	_RS uint16_t reserved4;
	_RW uint16_t CRCPR;          // CRC polynomial register
	_RS uint16_t reserved5;
	_RO uint16_t RXCRCR;         // RX CRC register
	_RS uint16_t reserved6;
	_RO uint16_t TXCRCR;         // TX CRC register
	_RS uint16_t reserved7;
};
static_assert(offsetof(struct SPI_t, TXCRCR) == 0x18, "Wrong definition");

#define SPI_CR1_BIDIMODE                   (   1 << 15)      // Bidirectional data mode enable
#define SPI_CR1_BIDIOE                     (   1 << 14)      // Output enable in bidirectional mode
#define SPI_CR1_CRCEN                      (   1 << 13)      // Hardware CRC calculation enable
#define SPI_CR1_CRCNEXT                    (   1 << 12)      // CRC transfer next
#define SPI_CR1_CRCL                       (   1 << 11)      // CRC length
#define SPI_CR1_RXONLY                     (   1 << 10)      // Receive only
#define SPI_CR1_SSM                        (   1 << 9)       // Software slave management
#define SPI_CR1_SSI                        (   1 << 8)       // Internal slave select
#define SPI_CR1_LSBFIRST                   (   1 << 7)       // Frame format
#define SPI_CR1_SPE                        (   1 << 6)       // SPI enable
#define SPI_CR1_BR                         (0x07 << 3)       // Baud rate control
#define SPI_CR1_MSTR                       (   1 << 2)       // Master selection
#define SPI_CR1_CPOL                       (   1 << 1)       // Clock polarity
#define SPI_CR1_CPHA                       (   1 << 0)       // Clock phase

#define SPI_CR2_LDMATX                     (   1 << 14)      // Last DMA transfer for transmission
#define SPI_CR2_LDMARX                     (   1 << 13)      // Last DMA transfer for reception
#define SPI_CR2_FRXTH                      (   1 << 12)      // FIFO reception threshold
#define SPI_CR2_DS                         (0x0F << 8)       // Data size
#define SPI_CR2_TXEIE                      (   1 << 7)       // TX buffer empty interrupt enable
#define SPI_CR2_RXNEIE                     (   1 << 6)       // RX buffer not empty interrupt enable
#define SPI_CR2_ERRIE                      (   1 << 5)       // Error interrupt enable
#define SPI_CR2_FRF                        (   1 << 4)       // Frame format
#define SPI_CR2_NSSP                       (   1 << 3)       // NSS pulse management
#define SPI_CR2_SSOE                       (   1 << 2)       // SS output enable
#define SPI_CR2_TXDMAEN                    (   1 << 1)       // TX buffer DMA enable
#define SPI_CR2_RXDMAEN                    (   1 << 0)       // RX buffer DMA enable

#define SPI_SR_FTLVL                       (0x03 << 11)      // FIFO Transmission Level
#define SPI_SR_FRLVL                       (0x03 << 9)       // FIFO reception level
#define SPI_SR_FRE                         (   1 << 8)       // Frame format error
#define SPI_SR_BSY                         (   1 << 7)       // Busy flag
#define SPI_SR_OVR                         (   1 << 6)       // Overrun flag
#define SPI_SR_MODF                        (   1 << 5)       // Mode fault
#define SPI_SR_CRCERR                      (   1 << 4)       // CRC error flag
#define SPI_SR_TXE                         (   1 << 1)       // Transmit buffer empty
#define SPI_SR_RXNE                        (   1 << 0)       // Receive buffer not empty

#define SPI_DR_DR                          (0xFFFF << 0)     // Data register

#define SPI_CRCPR_CRCPOLY                  (0xFFFF << 0)     // CRC polynomial register

#define SPI_RXCRCR_RXCRC                   (0xFFFF << 0)     // RX CRC register

// == I2C =============================================================================================================

struct I2C_t {
	_RW uint32_t CR1;            // Control register 1
	_RW uint32_t CR2;            // Control register 2
	_RW uint32_t OAR1;           // Own address register 1
	_RW uint32_t OAR2;           // Own address register 2
	_RW uint32_t TIMINGR;        // Timing register
	_RW uint32_t TIMEOUTR;       // Timeout register
	_RW uint32_t ISR;            // Interrupt and status register
	_RW uint32_t ICR;            // Interrupt clear register
	_RW uint8_t  PECR;           // PEC register
	_RS uint8_t  reserved1[3];
	_RW uint8_t  RXDR;           // Receive data register
	_RS uint8_t  reserved2[3];
	_RW uint8_t  TXDR;           // Transmit data register
	_RS uint8_t  reserved3[3];
};
static_assert(offsetof(struct I2C_t, TXDR) == 0x28, "Wrong definition");

#define I2C_CR1_PECEN                      (   1 << 23)      // PEC enable
#define I2C_CR1_ALERTEN                    (   1 << 22)      // SMBus alert enable
#define I2C_CR1_SMBDEN                     (   1 << 21)      // SMBus Device Default address enable
#define I2C_CR1_SMBHEN                     (   1 << 20)      // SMBus Host address enable
#define I2C_CR1_GCEN                       (   1 << 19)      // General call enable
#define I2C_CR1_NOSTRETCH                  (   1 << 17)      // Clock stretching disable
#define I2C_CR1_SBC                        (   1 << 16)      // Slave byte control
#define I2C_CR1_RXDMAEN                    (   1 << 15)      // DMA reception requests enable
#define I2C_CR1_TXDMAEN                    (   1 << 14)      // DMA transmission requests enable
#define I2C_CR1_ANFOFF                     (   1 << 12)      // Analog noise filter OFF
#define I2C_CR1_DNF                        (0x0F << 8)       // Digital noise filter
#define I2C_CR1_ERRIE                      (   1 << 7)       // Error interrupts enable
#define I2C_CR1_TCIE                       (   1 << 6)       // Transfer Complete interrupt enable
#define I2C_CR1_STOPIE                     (   1 << 5)       // STOP detection Interrupt enable
#define I2C_CR1_NACKIE                     (   1 << 4)       // Not acknowledge received Interrupt enable
#define I2C_CR1_ADDRIE                     (   1 << 3)       // Address match Interrupt enable (slave only)
#define I2C_CR1_RXIE                       (   1 << 2)       // RX Interrupt enable
#define I2C_CR1_TXIE                       (   1 << 1)       // TX Interrupt enable
#define I2C_CR1_PE                         (   1 << 0)       // Peripheral enable

#define I2C_CR2_PECBYTE                    (   1 << 26)      // Packet error checking byte
#define I2C_CR2_AUTOEND                    (   1 << 25)      // Automatic end mode (master mode)
#define I2C_CR2_RELOAD                     (   1 << 24)      // NBYTES reload mode
#define I2C_CR2_NBYTES                     (0xFF << 16)      // Number of bytes
#define I2C_CR2_NACK                       (   1 << 15)      // NACK generation (slave mode)
#define I2C_CR2_STOP                       (   1 << 14)      // Stop generation (master mode)
#define I2C_CR2_START                      (   1 << 13)      // Start generation
#define I2C_CR2_HEAD10R                    (   1 << 12)      // 10-bit address header only read direction (master receiver mode)
#define I2C_CR2_ADD10                      (   1 << 11)      // 10-bit addressing mode (master mode)
#define I2C_CR2_RDWRN                      (   1 << 10)      // Transfer direction (master mode)
#define I2C_CR2_SADD98                     (0x03 << 1)       // Slave address bit 9:8 (master mode)
#define I2C_CR2_SADD                       (0xFF << 1)       // Slave address bit 7:1 (master mode)
#define I2C_CR2_SADD0                      (   1 << 0)       // Slave address bit 0 (master mode)

#define I2C_OAR1_OA1EN                     (   1 << 15)      // Own Address 1 enable
#define I2C_OAR1_OA1MODE                   (   1 << 10)      // Own Address 1 10-bit mode
#define I2C_OAR1_OA198                     (0x03 << 8)       // Interface address, bits 9:8 in 10-bit addressing mode
#define I2C_OAR1_OA1                       (0x7F << 1)       // Interface address, bits 7:1
#define I2C_OAR1_OA10                      (   1 << 0)       // Interface address, bit 0 in 10-bit addressing mode

#define I2C_OAR2_OA2EN                     (   1 << 1)       // Own Address 2 enable
#define I2C_OAR2_OA2MSK                    (0x07 << 1)       // Own Address 2 masks
#define I2C_OAR2_ADD2                      (0x7F << 1)       // Interface address

#define I2C_TIMINGR_PRESC                  (0x0F << 28)      // Timing prescaler
#define I2C_TIMINGR_SCLDEL                 (0x0F << 20)      // Data setup time
#define I2C_TIMINGR_SDADEL                 (0x0F << 16)      // Data hold time
#define I2C_TIMINGR_SCLH                   (0xFF << 8)       // SCL high period (master mode)
#define I2C_TIMINGR_SCLL                   (0xFF << 0)       // SCL low period (master mode)

#define I2C_TIMEOUTR_TEXTEN                (   1 << 31)      // Extended clock timeout enable
#define I2C_TIMEOUTR_TIMEOUTB              (0x0FFF << 16)    // Bus timeout B
#define I2C_TIMEOUTR_TIMOUTEN              (   1 << 15)      // Clock timeout enable
#define I2C_TIMEOUTR_TIDLE                 (   1 << 12)      // Idle clock timeout detection
#define I2C_TIMEOUTR_TIMEOUTA              (0x0FFF << 0)     // Bus Timeout A

#define I2C_ISR_ADDCODE                    (0x7F << 17)      // Address match code (Slave mode)
#define I2C_ISR_DIR                        (   1 << 16)      // Transfer direction (Slave mode)
#define I2C_ISR_BUSY                       (   1 << 15)      // Bus busy
#define I2C_ISR_ALERT                      (   1 << 13)      // SMBus alert
#define I2C_ISR_TIMEOUT                    (   1 << 12)      // Timeout or Tlow detection flag
#define I2C_ISR_PECERR                     (   1 << 11)      // PEC Error in reception
#define I2C_ISR_OVR                        (   1 << 10)      // Overrun/Underrun (slave mode)
#define I2C_ISR_ARLO                       (   1 << 9)       // Arbitration lost
#define I2C_ISR_BERR                       (   1 << 8)       // Bus error
#define I2C_ISR_TCR                        (   1 << 7)       // Transfer Complete Reload
#define I2C_ISR_TC                         (   1 << 6)       // Transfer Complete (master mode)
#define I2C_ISR_STOPF                      (   1 << 5)       // Stop detection flag
#define I2C_ISR_NACKF                      (   1 << 4)       // Not Acknowledge received flag
#define I2C_ISR_ADDR                       (   1 << 3)       // Address matched (slave mode)
#define I2C_ISR_RXNE                       (   1 << 2)       // Receive data register not empty (receivers)
#define I2C_ISR_TXIS                       (   1 << 1)       // Transmit interrupt status (transmitters)
#define I2C_ISR_TXE                        (   1 << 0)       // Transmit data register empty (transmitters)

#define I2C_ICR_ALERTCF                    (   1 << 13)      // Alert flag clear
#define I2C_ICR_TIMOUTCF                   (   1 << 12)      // Timeout detection flag clear
#define I2C_ICR_PECCF                      (   1 << 11)      // PEC Error flag clear
#define I2C_ICR_OVRCF                      (   1 << 10)      // Overrun/Underrun flag clear
#define I2C_ICR_ARLOCF                     (   1 << 9)       // Arbitration Lost flag clear
#define I2C_ICR_BERRCF                     (   1 << 8)       // Bus error flag clear
#define I2C_ICR_STOPCF                     (   1 << 5)       // Stop detection flag clear
#define I2C_ICR_NACKCF                     (   1 << 4)       // Not Acknowledge flag clear
#define I2C_ICR_ADDRCF                     (   1 << 3)       // Address Matched flag clear

#define I2C_PECR_PEC                       (0xFF << 0)       // Packet error checking register

#define I2C_RXDR_RXDATA                    (0xFF << 0)       // 8-bit receive data

#define I2C_TXDR_TXDATA                    (0xFF << 0)       // 8-bit transmit data

// == USART ===========================================================================================================

struct USART_t {
	_RW uint32_t CR1;            // Control register 1
	_RW uint32_t CR2;            // Control register 2
	_RW uint32_t CR3;            // Control register 3
	_RW uint32_t BRR;            // Baud rate register
	_RS uint32_t GTPR;           // Guard time and prescaler register
	_RW uint32_t RTOR;           // Receiver timeout register
	_RW uint32_t RQR;            // Request register
	_RW uint32_t ISR;            // Interrupt and status register
	_RW uint32_t ICR;            // Interrupt flag clear register
	_RW uint8_t  RDR;            // Receive data register
	_RS uint8_t  reserved1[3];
	_RW uint8_t  TDR;            // Transmit data register
	_RS uint8_t  reserved2[3];
};
static_assert(offsetof(struct USART_t, TDR) == 0x28, "Wrong definition");

#define USART_CR1_M1                       (   1 << 28)      // Word length, together with M0
#define USART_CR1_EOBIE                    (   1 << 27)      // End of Block interrupt enable
#define USART_CR1_RTOIE                    (   1 << 26)      // Receiver timeout interrupt enable
#define USART_CR1_DEAT                     (0x1F << 21)      // Driver Enable assertion time
#define USART_CR1_DEDT                     (0x1F << 16)      // Driver Enable de-assertion time
#define USART_CR1_OVER8                    (   1 << 15)      // Oversampling mode
#define USART_CR1_CMIE                     (   1 << 14)      // Character match interrupt enable
#define USART_CR1_MME                      (   1 << 13)      // Mute mode enable
#define USART_CR1_M0                       (   1 << 12)      // Word length
#define USART_CR1_WAKE                     (   1 << 11)      // Receiver wakeup method
#define USART_CR1_PCE                      (   1 << 10)      // Parity control enable
#define USART_CR1_PS                       (   1 << 9)       // Parity selection
#define USART_CR1_PEIE                     (   1 << 8)       // PE interrupt enable
#define USART_CR1_TXEIE                    (   1 << 7)       // Tx interrupt enable
#define USART_CR1_TCIE                     (   1 << 6)       // Transmission complete interrupt enable
#define USART_CR1_RXNEIE                   (   1 << 5)       // RXNE interrupt enable
#define USART_CR1_IDLEIE                   (   1 << 4)       // IDLE interrupt enable
#define USART_CR1_TE                       (   1 << 3)       // Transmitter enable
#define USART_CR1_RE                       (   1 << 2)       // Receiver enable
#define USART_CR1_UE                       (   1 << 0)       // USART enable

#define USART_CR2_ADD2                     (0x0F << 28)      // Address of the USART node [7:4]
#define USART_CR2_ADD1                     (0x0F << 24)      // Address of the USART node [3:0]
#define USART_CR2_RTOEN                    (   1 << 23)      // Receiver timeout enable
#define USART_CR2_ABRMOD                   (0x03 << 21)      // Auto baud rate mode
#define USART_CR2_ABREN                    (   1 << 20)      // Auto baud rate enable
#define USART_CR2_MSBFIRST                 (   1 << 19)      // Most significant bit first
#define USART_CR2_DATAINV                  (   1 << 18)      // Binary data inversion
#define USART_CR2_TXINV                    (   1 << 17)      // TX pin active level inversion
#define USART_CR2_RXINV                    (   1 << 16)      // RX pin active level inversion
#define USART_CR2_SWAP                     (   1 << 15)      // Swap TX/RX pins
#define USART_CR2_STOP                     (0x03 << 12)      // STOP bits
#define USART_CR2_CLKEN                    (   1 << 11)      // Clock enable
#define USART_CR2_CPOL                     (   1 << 10)      // Clock polarity
#define USART_CR2_CPHA                     (   1 << 9)       // Clock phase
#define USART_CR2_LBCL                     (   1 << 8)       // Last bit clock pulse
#define USART_CR2_ADDM7                    (   1 << 4)       // 7-bit Address Detection/4-bit Address Detection

#define USART_CR3_DEP                      (   1 << 15)      // Driver enable polarity selection
#define USART_CR3_DEM                      (   1 << 14)      // Driver enable mode
#define USART_CR3_DDRE                     (   1 << 13)      // DMA disable on reception Error
#define USART_CR3_OVRDIS                   (   1 << 12)      // Overrun disable
#define USART_CR3_ONEBIT                   (   1 << 11)      // One sample bit method enable
#define USART_CR3_CTSIE                    (   1 << 10)      // CTS interrupt enable
#define USART_CR3_CTSE                     (   1 << 9)       // CTS enable
#define USART_CR3_RTSE                     (   1 << 8)       // RTS enable
#define USART_CR3_DMAT                     (   1 << 7)       // DMA enable transmitter
#define USART_CR3_DMAR                     (   1 << 6)       // DMA enable receiver
#define USART_CR3_HDSEL                    (   1 << 3)       // Half-duplex selection
#define USART_CR3_EIE                      (   1 << 0)       // Error interrupt enable

#define USART_BRR_DIVMANTISSA              (0x0FFF << 4)     // Mantissa of USARTDIV
#define USART_BRR_DIVFRACTION              (0x0F << 0)       // Fraction of USARTDIV

#define USART_RTOR_RTO                     (0x00FFFFFF << 0) // Receiver timeout value

#define USART_RQR_RXFRQ                    (   1 << 3)       // Receive data flush request
#define USART_RQR_MMRQ                     (   1 << 2)       // Mute mode request
#define USART_RQR_SBKRQ                    (   1 << 1)       // Send break request
#define USART_RQR_ABRRQ                    (   1 << 0)       // Auto baud rate request

#define USART_ISR_SBKF                     (   1 << 18)      // Send break flag
#define USART_ISR_CMF                      (   1 << 17)      // Character match flag
#define USART_ISR_BUSY                     (   1 << 16)      // Busy flag
#define USART_ISR_ABRF                     (   1 << 15)      // Auto baud rate flag
#define USART_ISR_ABRE                     (   1 << 14)      // Auto baud rate error
#define USART_ISR_RTOF                     (   1 << 11)      // Receiver timeout
#define USART_ISR_CTS                      (   1 << 10)      // CTS flag
#define USART_ISR_CTSIF                    (   1 << 9)       // CTS interrupt flag
#define USART_ISR_TXE                      (   1 << 7)       // Transmit data register empty
#define USART_ISR_TC                       (   1 << 6)       // Transmission complete
#define USART_ISR_RXNE                     (   1 << 5)       // Read data register not empty
#define USART_ISR_IDLE                     (   1 << 4)       // Idle line detected
#define USART_ISR_ORE                      (   1 << 3)       // Overrun error
#define USART_ISR_NF                       (   1 << 2)       // START bit noise detection flag
#define USART_ISR_FE                       (   1 << 1)       // Framing error
#define USART_ISR_PE                       (   1 << 0)       // Parity error

#define USART_ICR_CMCF                     (   1 << 17)      // Character match clear flag
#define USART_ICR_RTOCF                    (   1 << 11)      // Receiver timeout clear flag
#define USART_ICR_CTSCF                    (   1 << 9)       // CTS clear flag
#define USART_ICR_TCCF                     (   1 << 6)       // Transmission complete clear flag
#define USART_ICR_IDLECF                   (   1 << 4)       // Idle line detected clear flag
#define USART_ICR_ORECF                    (   1 << 3)       // Overrun error clear flag
#define USART_ICR_NCF                      (   1 << 2)       // Noise detected clear flag
#define USART_ICR_FECF                     (   1 << 1)       // Framing error clear flag
#define USART_ICR_PECF                     (   1 << 0)       // Parity error clear flag

#define USART_RDR_RDR                      (0xFF << 0)       // Receive data value

#define USART_TDR_TDR                      (0xFF << 0)       // Transmit data value

// == DBGMCU ==========================================================================================================

struct DBGMCU_t {
	_RO uint32_t IDCODE;         // MCU device ID code
	_RW uint32_t CR;             // Debug MCU configuration register
	_RW uint32_t APB1_FZ;        // APB1 freeze register
	_RW uint32_t APB2_FZ;        // APB2 freeze register
};
static_assert(offsetof(struct DBGMCU_t, CR) == 0x04, "Wrong definition");

#define DBGMCU_IDCODE_REVID                (0xFFFF << 16)    // Revision identifier
#define DBGMCU_IDCODE_DEVID                (0x0FFF << 0)     // Device identifier

// == FLASH ===========================================================================================================

struct FLASH_t {
	_RW uint32_t ACR;            // Access control register
	_WO uint32_t KEYR;           // FPEC key register
	_WO uint32_t OPTKEYR;        // OPTKEY register
	_RW uint32_t SR;             // Status register
	_RW uint32_t CR;             // Control register
	_WO uint32_t AR;             // Address register
	_RS uint32_t reserved1;
	_RO uint32_t OBR;            // Option byte register
	_RO uint32_t WRPR;           // Write protection register
};
static_assert(offsetof(struct FLASH_t, WRPR) == 0x20, "Wrong definition");

#define FLASH_ACR_PRFTBS                   (   1 << 5)       // Prefetch buffer status
#define FLASH_ACR_PRFTBE                   (   1 << 4)       // Prefetch buffer enable
#define FLASH_ACR_LATENCY                  (0x07 << 0)       // Latency

#define FLASH_KEYR_FKEYR                   (0xFFFFFFFF << 0) // FPEC key

#define FLASH_OPTKEYR_OPTKEYR              (0xFFFFFFFF << 0) // Option byte key

#define FLASH_SR_EOP                       (   1 << 5)       // End of operation
#define FLASH_SR_WRPRTERR                  (   1 << 4)       // Write protection error
#define FLASH_SR_PGERR                     (   1 << 2)       // Programming error
#define FLASH_SR_BSY                       (   1 << 0)       // Busy

#define FLASH_CR_OBLLAUNCH                 (   1 << 13)      // Force option byte loading
#define FLASH_CR_EOPIE                     (   1 << 12)      // End of operation interrupt enable
#define FLASH_CR_ERRIE                     (   1 << 10)      // Error interrupt enable
#define FLASH_CR_OPTWRE                    (   1 << 9)       // Option bytes write enable
#define FLASH_CR_LOCK                      (   1 << 7)       // Lock
#define FLASH_CR_STRT                      (   1 << 6)       // Start
#define FLASH_CR_OPTER                     (   1 << 5)       // Option byte erase
#define FLASH_CR_OPTPG                     (   1 << 4)       // Option byte programming
#define FLASH_CR_MER                       (   1 << 2)       // Mass erase
#define FLASH_CR_PER                       (   1 << 1)       // Page erase
#define FLASH_CR_PG                        (   1 << 0)       // Programming

#define FLASH_AR_FAR                       (0xFFFFFFFF << 0) // Flash Address

#define FLASH_OBR_DATA1                    (0xFF << 18)      // Data 1
#define FLASH_OBR_DATA0                    (0xFF << 16)      // Data 0
#define FLASH_OBR_OPTRAMPARITYCHECK        (   1 << 14)      // User option byte: RAM_PARITY_CHECK
#define FLASH_OBR_OPTVDDAMONITOR           (   1 << 13)      // User option byte: VDDA_MONITOR
#define FLASH_OBR_OPTBOOT1                 (   1 << 12)      // User option byte: nBOOT1
#define FLASH_OBR_USERSTDBY                (   1 << 10)      // nRST_STDBY
#define FLASH_OBR_USERSTOP                 (   1 << 9)       // nRST_STOP
#define FLASH_OBR_USERWDG                  (   1 << 8)       // WDG_SW
#define FLASH_OBR_RDPRT                    (0x03 << 1)       // Read protection
#define FLASH_OBR_OPTERR                   (   1 << 0)       // Option byte error

#define FLASH_WRPR_WRP                     (0xFFFFFFFF << 0) // Write protect

