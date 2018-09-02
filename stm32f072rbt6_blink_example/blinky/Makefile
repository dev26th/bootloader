# -
# Copyright (c) 2017, Lukasz Marcin Podkalicki <lpodkalicki@gmail.com>
# -

TARGET=main
STMDIR=lib

###################################################

CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
SE=arm-none-eabi-size
SF=st-flash

###################################################

STMSRCDIR = $(STMDIR)/Drivers/STM32F0xx_HAL_Driver/Src
vpath %.c $(STMSRCDIR)

INC_DIRS = $(STMDIR)/Drivers/STM32F0xx_HAL_Driver/Inc
INC_DIRS += $(STMDIR)/Drivers/CMSIS/Device/ST/STM32F0xx/Include
INC_DIRS += $(STMDIR)/Drivers/CMSIS/Include
INC_DIRS += .

INCLUDE = $(addprefix -I,$(INC_DIRS))

###################################################

CFLAGS  = -std=gnu99 -g -O2 -Wall -TSTM32F072RB_FLASH.ld
#CFLAGS  = -std=gnu99 -g -O2 -Wall -Tsimple.ld
CFLAGS += -mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m0
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
CFLAGS += -DUSE_STDPERIPH_DRIVER -DSTM32F072xB

###################################################

SRCS =  main.c \
	stm32f0xx_it.c \
	system_stm32f0xx.c
	
SRCS += stm32f0xx_hal.c \
	stm32f0xx_hal_rcc.c \
	stm32f0xx_hal_gpio.c \
	stm32f0xx_hal_cortex.c \
	startup_stm32f072xb.s

###################################################

.PHONY: $(TARGET)

$(TARGET): $(TARGET).elf

$(TARGET).elf: $(SRCS)
	$(CC) $(INCLUDE) $(CFLAGS) $^ -o $@ 
	$(CP) -O ihex $(TARGET).elf $(TARGET).hex
	$(CP) -O binary $(TARGET).elf $(TARGET).bin

clean:
	rm -f *.o $(TARGET).elf $(TARGET).hex $(TARGET).bin

flash: 
	$(SF) write $(TARGET).bin 0x8000000
