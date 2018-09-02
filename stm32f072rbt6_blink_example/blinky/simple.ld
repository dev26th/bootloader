OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")

MEMORY
{
    rom (rx)  : ORIGIN = 0x08001000, LENGTH = 0x00003000
}

SECTIONS
{
    .text :
    {
        KEEP(*(.isr_vector))
    } > rom
}
