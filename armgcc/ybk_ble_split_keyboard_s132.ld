/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x26000, LENGTH = 0x5a000
  RAM (rwx) :  ORIGIN = 0x20002B00, LENGTH = 0xd500
}

INCLUDE "ybk_usbd_ble_split_keyboard.ld"