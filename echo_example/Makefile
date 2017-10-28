all:
	mkdir -p build
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -flto -Wall -Wextra -Wpointer-arith -Wlogical-op -Wfloat-equal -I include -fabi-version=0 -fno-exceptions -Wabi -nostartfiles -Xlinker --gc-sections --specs=nano.specs -T ld/mem.ld -T ld/arm-gcc-link_c.ld -o build/main.elf main.c && arm-none-eabi-objcopy -O binary build/main.elf build/main.bin && arm-none-eabi-size build/main.elf

clean:
	rm -rf build/*

.PHONY: all clean

