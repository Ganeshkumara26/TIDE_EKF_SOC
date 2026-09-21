.section .text.start
.globl _start
_start:
  li sp, 0x20000
  call main
  li t0, 0x10000008
  sw zero, 0(t0)
1: j 1b
