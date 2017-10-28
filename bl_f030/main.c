#include <stdbool.h>
#include <assert.h>
#include <string.h>

#include "stm32f030_f4.h"
#include "startup-common.h"
#include "isr/stm32f030.h"

#include "tiny-AES-c/aes.h"

#define PRODUCT_ID       0xAABBCCDD
static uint8_t KEY[] = { 0xfe, 0xcc, 0xb8, 0x70, 0x05, 0xda, 0x13, 0x0c, 0x06, 0xe8, 0x6d, 0xd9, 0xf1, 0x75, 0x9d, 0x45 };

#define START_TIMEOUT            10 // in ticks, 1 tick is 100 ms
#define COMM_TIMEOUT             10 // in ticks
#define CPU_F               8000000
#define BAUD                 115200
#define APP_START        0x08001000 // 4k for bootloader
#define APP_END          0x08004000
#define VTOR_SIZE              0xC0
#define PAGE_SIZE              1024
#define BLOCK_SIZE               16
#define PROTOCOL_VERSION 0x00000001

#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xCDEF89AB

// ==================================================================================================================================================

enum Command {
    Command_GetVersion = 0x01,
    Command_Start      = 0x02,
    Command_NextPage   = 0x03,
    Command_Reset      = 0x04,

    Command_OK         = 0x40,
    Command_Error      = 0x80,
};

enum CommState {
    CommState_Start,
    CommState_Ready,
    CommState_NextByte,
    CommState_Done
};

struct StartData {
    uint32_t protocolVersion;
    uint32_t productId;
    uint32_t appVersion;
    uint32_t pageCount;
    uint8_t  iv[BLOCK_SIZE];
    uint32_t crc;
};

static volatile uint32_t timeTick;

static volatile enum Command     command;
static volatile enum CommState   commState;
static volatile uint8_t          buf[PAGE_SIZE];
static volatile struct StartData startData;
static volatile uint8_t*         bufPos;
static volatile uint32_t         pageTail;

static_assert(sizeof(struct StartData) <= sizeof(buf), "Data too big");

static uint32_t timeout;
static uint32_t pagePos;
static uint32_t pageRem;
static uint32_t appFirstWord;
static uint32_t crc;

// ==================================================================================================================================================

static uint32_t revU32(uint32_t d) {
    uint32_t r = 0;
    for(size_t i = 0; i < 32; ++i) {
        r <<= 1;
        r |= (d & 0x01);
        d >>= 1;
    }
    return r;
}

uint32_t CRC_init() {
    return 0xFFFFFFFF;
}

uint32_t CRC_addByte(uint32_t crc, uint8_t b) {
    for(uint8_t pos = 0x01; pos; pos <<= 1) {
        uint32_t msb = crc & 0x80000000;
        if(b & pos) msb ^= 0x80000000;

        if(msb) crc = (crc << 1) ^ 0x4C11DB7;
        else    crc = (crc << 1);
    }
    return crc;
}

uint32_t CRC_add(uint32_t crc, uint8_t const* data, size_t dataLen) {
    for(; dataLen > 0; ++data, --dataLen) {
        crc = CRC_addByte(crc, *data);
    }
    return crc;
}

uint32_t CRC_result(uint32_t crc) {
    return (revU32(crc) ^ 0xFFFFFFFF);
}

// ==================================================================================================================================================

void FLASH_waitBusy(void) {
    while(FLASH->SR & FLASH_SR_BSY) {}
}

void FLASH_unlock(void) {
    FLASH_waitBusy();

    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;

    FLASH_waitBusy();
}

void FLASH_lock(void) {
    FLASH_waitBusy();
    FLASH->CR |= FLASH_CR_LOCK;
    FLASH_waitBusy();
}

// addr may be any one for the page to erase
void FLASH_erasePage(uint32_t addr) {
    FLASH_waitBusy();

    FLASH->CR |= FLASH_CR_PER; // one page
    FLASH->AR = addr;
    FLASH->CR |= FLASH_CR_STRT; // start
    asm volatile(       // Wait for busy-flag. See STM32F100x4/6/8/B Errata sheet, 2.9
        " nop " "\n\t"
        " nop " "\n\t"
    );

    FLASH_waitBusy();
    FLASH->CR &= ~FLASH_CR_PER;
}

void FLASH_write(uint32_t addr, uint32_t const * data, size_t dataLen) {
    FLASH_waitBusy();
    FLASH->CR |= FLASH_CR_PG; // programming
    FLASH_waitBusy();

    for(; dataLen > 0; ++data, --dataLen) {
        uint32_t v = *data;
        *(volatile uint16_t*)addr = (uint16_t)v;
        FLASH_waitBusy();

        addr += 2;
        v >>= 16;
        *(volatile uint16_t*)addr = (uint16_t)v;
        FLASH_waitBusy();

        addr += 2;
    }

    FLASH->CR &= ~FLASH_CR_PG;
}

// ==================================================================================================================================================

bool UART_transmissionComplete(void) {
    return (USART1->ISR & USART_ISR_TC);
}

void UART_sendByte(uint8_t b) {
    while(!(USART1->ISR & USART_ISR_TXE)) {}

    USART1->TDR = b;
}

void UART_send(const uint8_t* d, size_t l) {
    for(; l > 0; ++d, --l)
        UART_sendByte(*d);
}

// ==================================================================================================================================================

void SysTick_Handler(void) {
    ++timeTick;
}

void USART1_IRQHandler(void) {
    if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE) {
        uint8_t c = USART1->RDR;
        switch(commState) {
            case CommState_Start:
                if(c == (uint8_t)Command_GetVersion) {
                    // fallthrough
                }
                else {
                    break; // ignore
                }

            case CommState_Ready:
                command = (enum Command)c;
                switch(command) {
                    case Command_Start:
                        bufPos = (uint8_t*)&startData;
                        pageTail = sizeof(startData);
                        commState = CommState_NextByte;
                        break;

                    case Command_NextPage:
                        bufPos = buf;
                        pageTail = sizeof(buf);
                        commState = CommState_NextByte;
                        break;

                    default:
                        commState = CommState_Done;
                }
                break;

            case CommState_NextByte:
                *(bufPos++) = c;
                if(--pageTail == 0)
                   commState = CommState_Done;
                break;

            case CommState_Done:
                break;
        }
    }
}

static bool isCorrectApp() {
    return (((*(const uint32_t*)APP_START) & 0x2FFE0000) == 0x20000000);
}

static void jumpToApp() {
    // copy app's vector table to RAM, map RAM to address 0
    __disable_irq();
    memcpy((void*)0x20000000, (const void*)APP_START, VTOR_SIZE);
    SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEMMODE) | SYSCFG_CFGR1_MEMMODE_SRAM;
    __enable_irq();

    __DSB();
    volatile uint32_t* appBegin = (volatile uint32_t*)APP_START;
    uint32_t stack = *appBegin;
    uint32_t start = *(appBegin+1);

    __asm volatile(
        "msr MSP, %0\n\t"        // set stack
        "bx  %1\n\t"             // start the app
        :
        : "r"(stack), "r"(start)
    );
}

static void initHardware(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_IOPAEN;

    // UART1 on PA9/PA10
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_UP | GPIO_PUPDR_PUPDR10_UP;
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9|GPIO_MODER_MODER10))
        | (GPIO_MODER_MODER9_ALT | GPIO_MODER_MODER10_ALT);
    GPIOA->AFRH = (GPIOA->AFRH &~ (GPIO_AFRH_AFR9 | GPIO_AFRH_AFR10))
        | (1 << (1 * 4)) | (1 << (2 * 4));

    USART1->BRR = CPU_F / BAUD;
    USART1->CR1 = USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;

    SysTick->RVR = CPU_F / 10 - 1; // 100 ms
    SysTick->CVR = 0;
    SysTick->CSR |= SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE;

    NVIC_EnableIRQ(USART1_IRQn);
}

static void deinitHardware(void) {
    NVIC_DisableIRQ(USART1_IRQn);

    // all changed registers to their reset-values
    SysTick->CSR = 0;

    USART1->CR1  = 0;
    USART1->BRR  = 0;

    GPIOA->MODER = 0x28000000;
    GPIOA->PUPDR = 0x24000000;
    GPIOA->AFRH  = 0;
    GPIOA->ODR   = 0;

    RCC->APB2ENR = 0;
    RCC->AHBENR  = 0x00000014;
}

static void doGetVersion(void) {
    uint32_t v = PROTOCOL_VERSION;
    uint32_t p = PRODUCT_ID;

    UART_sendByte((uint8_t)Command_OK | (uint8_t)Command_GetVersion);
    UART_send((const uint8_t*)&v, sizeof(v));
    UART_send((const uint8_t*)&p, sizeof(p));
}

static void doStart(void) {
    if(startData.protocolVersion != PROTOCOL_VERSION || startData.productId != PRODUCT_ID) {
        UART_sendByte((uint8_t)Command_Error | (uint8_t)Command_Start);
        return;
    }

    FLASH_unlock();
    for(uint32_t a = APP_START; a < APP_END; a += PAGE_SIZE)
        FLASH_erasePage(a);
    FLASH_lock();
    pagePos = APP_START;
    pageRem = startData.pageCount;

    AES_CBC_init(KEY, (const uint8_t *)startData.iv);
    crc = CRC_init();

    UART_sendByte((uint8_t)Command_OK | (uint8_t)Command_Start);
}

static void doNextPage(void) {
    if(pagePos < APP_START || pagePos >= APP_END || !pageRem) {
        UART_sendByte((uint8_t)Command_Error | (uint8_t)Command_NextPage);
        return;
    }

    AES_CBC_decrypt_buffer((uint8_t *)buf, sizeof(buf));
    crc = CRC_add(crc, (uint8_t *)buf, sizeof(buf));

    const uint32_t* data = (const uint32_t*)buf;
    uint32_t writePos = pagePos;
    size_t writeLen = sizeof(buf)/4;

    if(pagePos == APP_START) { // skip first 4 bytes of the app
        appFirstWord = *data;
        ++data;
        writePos += sizeof(uint32_t);
        --writeLen;
    }

    bool res = true;
    FLASH_unlock();

    FLASH_write(writePos, data, writeLen);
    pagePos += PAGE_SIZE;
    if(--pageRem == 0) { // end of transfer, validate and write the skipped 4 bytes
        crc = CRC_result(crc);
        if(crc != startData.crc) {
            res = false;
        }
        else {
            FLASH_write(APP_START, &appFirstWord, 1);
        }
    }
    FLASH_lock();

    if(res)
        UART_sendByte((uint8_t)Command_OK | (uint8_t)Command_NextPage);
    else
        UART_sendByte((uint8_t)Command_Error | (uint8_t)Command_NextPage);
}

static void doReset(void) {
    UART_sendByte((uint8_t)Command_OK | (uint8_t)Command_Reset);
    while(!UART_transmissionComplete()) {}
    NVIC_SystemReset();
}

int main(void) {
    commState = CommState_Start;
    timeout = START_TIMEOUT;
    initHardware();

    while(1) {
        if(timeTick >= timeout) {
            if(commState == CommState_Start && isCorrectApp()) {
                deinitHardware();
                jumpToApp();
            }
            else {
                doReset();
            }
        }

        if(commState == CommState_Done) {
            switch(command) {
                case Command_GetVersion: doGetVersion(); break;
                case Command_Start:      doStart();      break;
                case Command_NextPage:   doNextPage();   break;
                case Command_Reset:      doReset();      break;
                default: UART_sendByte((uint8_t)Command_Error);
            }
            commState = CommState_Ready;
            timeout = timeTick + COMM_TIMEOUT;
        }
    }

    return 0;
}

