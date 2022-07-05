/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0xED000 - 0x26000
  RAM (rwx) :  ORIGIN = 0x20010000, LENGTH = 0x30000
}

SECTIONS
{
}

INCLUDE "ybk_usbd_ble_split_keyboard.ld"