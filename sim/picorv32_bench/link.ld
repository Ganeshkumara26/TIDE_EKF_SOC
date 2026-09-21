ENTRY(_start)
MEMORY { RAM (rwx) : ORIGIN = 0, LENGTH = 128K }
SECTIONS {
  .text : { *(.text.start) *(.text*) *(.rodata*) } > RAM
  .data : { *(.data*) *(.sdata*) } > RAM
  .bss  : { *(.bss*) *(.sbss*) *(COMMON) } > RAM
}
