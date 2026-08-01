MEMORY
{
  /* STM32WBA65RI Flash: 2048KB (2MB), RAM: 512KB */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM   (rwx): ORIGIN = 0x20000000, LENGTH = 512K
}

/* Location of the stack pointer at reset */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
