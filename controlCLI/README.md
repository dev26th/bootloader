Firmware Loader
===============

sudo ./controlCLI /dev/_UART_ FIRMWARE_CRYPTO.bin


Building
--------

Linux/Mac OS X:
> make


Example:
--------
```cpp
serra82@gmail ~/bootloader/controlCLI $ ./controlCLI /dev/ttyUSB0 ../stm32f072rbt6_blink_example/blinky/mainBlu_crypto.bin 
file size:8228
Connecting..
Command_OK | Command_GetVersion  A  65

Connected
PROTOCOL_VERSION: 1000
PRODUCT_ID: ddccbbaa

Starting
Send start cmd
read from file:36
ERROR;65
Resend Command_Start for reply 65
ERROR;1
ERROR;0
ERROR;0
ERROR;0
ERROR;221
ERROR;204
ERROR;187
ERROR;170

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:1024

Sending
read from file:0
Upgrade Finished
```c
