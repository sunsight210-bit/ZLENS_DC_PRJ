/* cmake/stm32f1xx.ld - STM32F103RCT6: 256KB Flash, 48KB SRAM */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); _etext = .; } >FLASH
  .data : AT(_etext) { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } >RAM
  .bss : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } >RAM
  ._user_heap_stack : {
    . = ALIGN(8);
    PROVIDE(end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
}
