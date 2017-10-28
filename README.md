# bootloader

A bootloader for STM32F030 with encryption + PC-tools. The bootloader works via UART. A firmware is encrypted with AES-128 and is protected against communication errors with CRC-32.

## Content
* *bl_f030* - the bootloader itself
* *creator* - PC tool to create an encrypted file with firmware
* *control* - PC tool to download the encrypted firmware into uC
* *led_example* - a very simple example firmware
* *echo_example* - a bit more real firmware

The PC-tools are developed with Qt and may be compiled for Windows (untested) or Linux (tested).

## How to use

### You as developer, one time

Make a copy of *bl_f030*, take a look into *main.c*. Here you should change the define *PRODUCT_ID* to something unique for each your project - just to avoid a user mistakes. In any case you must change the *KEY* to random 16 bytes.

You might want to change the condition to start the bootloader too. Default - wait 1 second for command from the *'control'*. Check any other hardware-specific options. The bootloader is configured for UART 115200 8N1, TX on PA9, RX on PA10.

Now compile and flash the bootloader. **Don't forget to set the read protection** (see RM0360, chapter 3.3.1).

### You as developer, each upgrade

Your app must start from 0x08001000 and leave first 192 bytes of RAM unused. E.g. see *echo_example/ld/mem.ld*.

After compiling your firmware, use *'creator'* to encrypt it. Enter *'Product ID'* and *'Key'* exactly as in your copy of *bl_f030*. *'App Version'* - anything, just to help your users to distinct upgrades.

### Your user

Provide *'control'* together with the encrypted firmware. 

The user has to:
1. Connect a device via UART (e.g. using UART-USB adapter).
2. Start the *'control'*.
3. Select corresponding UART interface and *'Connect'*.
4. Restart the device.
5. Now the *'control'* should connect to the device and show its Product-ID and protocol version.
6. Select the encrypted firmware in *'Input file'*. If the device matches the firmware, The *'Download'* button will be enabled.
8. Click *'Download'* and wait for success message (a few seconds).
9. *'Disconnect'* - the device is upgraded.
