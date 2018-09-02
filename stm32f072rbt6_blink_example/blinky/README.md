# Simple Blinky
## Board
[STM32F072RB-Disovery](http://www.st.com/resource/en/data_brief/32f072bdiscovery.pdf)

## MCU
[STM32F042F6P6](http://www.st.com/web/en/resource/technical/document/datasheet/DM00105814.pdf)

## Library
[STM32Cube_FW_F0_V1.4.0](http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/LN1897/PF260612?s_searchtype=partnumber)

## Configuration

### LED:
 * GPIO of port C:
  * GREEN_LED -> PC9

## Compiling and Burning (arm-none-eabi toolchain)
```
$ make && make flash
```
