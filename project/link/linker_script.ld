ENTRY(Reset_Handler)
OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)

MEMORY
{
    FLASH(xr) : ORIGIN = 0x08000000, LENGTH = 1024K /* Aliased to 0x00000000 if boot from Flash selected (BOOT0 pin = 0). */
    RAM(xrw) :  ORIGIN = 0x20000000, LENGTH = 112K  /* SRAM 1 */
}

_stack_top_addr = ORIGIN(RAM) + LENGTH(RAM); /* end of Ram will be the highest stack addr - the stack grows downwards from there */

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
    }>FLASH
    
    .text :
    {
        . = ALIGN(4);
        _text_start_addr = .;   /* create a global symbol at text start */
        *(.text)
        *(.text*)               /* e.g. .text.Reset_Handler */
        . = ALIGN(4);
        _text_end_addr = .;     /* create a global symbol at text end */
    }>FLASH
    
    .rodata : /* Constant data (constants, strings, etc.) into Flash */
    {
        . = ALIGN(4);
        _rodata_start_addr = .; /* create a global symbol at rodata start */
        *(.rodata)
        *(.rodata*) 
        . = ALIGN(4);
        _rodata_end_addr = .;   /* create a global symbol at rodata end */
    }>FLASH
    
    _data_phys_addr = LOADADDR(.data); /* LOADADDR returns the absolute load address of the named section - in this case the Flash addr */
                                      /* _data_phys_addr will be used in startup as the source addr to load .data from Flash to Ram */

    .data : /* Initialized data section - resolve as if in Ram, actually stored in Flash, will be copied to Ram at startup */
    {
        . = ALIGN(4);
        _data_start_addr = .;   /* create a global symbol at data start */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _data_end_addr = .;     /* create a global symbol at data end */
    }>RAM AT>FLASH 
    
    .bss :  /* Uninitialized data section into Ram memory */
    {
        . = ALIGN (4);
        _bss_start_addr = .;    /* create a global symbol at bss start */
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _bss_end_addr = .;      /* create a global symbol at bss end */
    }>RAM
}
