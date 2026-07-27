/* Linker memory layout for STM32F411CEUx (512K flash, 128K SRAM).
 *
 * For a different chip, update ORIGIN/LENGTH from the datasheet.
 * Note: H7 parts have multiple RAM regions (DTCM, AXI SRAM, etc.);
 * you'll want to define them separately and be deliberate about
 * where the stack, control state, and ML buffers live.
 */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}
