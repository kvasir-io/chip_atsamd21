/* Linker script for ATSAMD21G18A cortex-m0plus */

MEMORY{
    flash  (xr ) : ORIGIN = 0x00000000, LENGTH = 256K
    ram    (xrw) : ORIGIN = 0x20000000, LENGTH =  32K
}

INCLUDE common_flash.ld
/*INCLUDE common_eeprom.ld   no RWW section on this part*/
INCLUDE common_ram.ld

INCLUDE common.ld
