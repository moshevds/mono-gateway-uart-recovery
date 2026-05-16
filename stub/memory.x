ENTRY(_start)

__ocram_origin = DEFINED(__ocram_origin) ? __ocram_origin : 0x10000000;
__ocram_length = DEFINED(__ocram_length) ? __ocram_length : 128K;

MEMORY
{
    OCRAM (rwx) : ORIGIN = __ocram_origin, LENGTH = __ocram_length
}

SECTIONS
{
    .text ORIGIN(OCRAM) : ALIGN(16)
    {
        KEEP(*(.text._start))
        *(.text .text.*)
        *(.rodata .rodata.*)
    } > OCRAM

    .data : ALIGN(16)
    {
        *(.data .data.*)
    } > OCRAM

    .bss (NOLOAD) : ALIGN(16)
    {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    } > OCRAM

    . = ALIGN(16);
    __image_end = .;
    __stack_top = ORIGIN(OCRAM) + LENGTH(OCRAM);

    /DISCARD/ :
    {
        *(.comment)
        *(.eh_frame*)
        *(.note*)
    }
}
