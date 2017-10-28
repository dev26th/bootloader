MEMORY
{
        rom (rx)  : ORIGIN = 0x08000000 + 0x1000, LENGTH = 16K - 0x1000
        ram (rwx) : ORIGIN = 0x20000000 + 0xC0,   LENGTH = 4K - 0xC0
}
