# ATSAMD21G18A: 256 KB flash, 32 KB RAM, no RWW section (so no EEPROM region), USB. (SAM D21 datasheet DS40001882,
# "Configuration Summary" and table 10-3 "SAM D21 Flash Memory Parameters".)
set(TARGET_MPU ATSAMD21G18A)
set(TARGET_FLASH_SIZE 262144)
set(TARGET_RAM_SIZE 32768)
set(TARGET_EEPROM_SIZE 0)

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/../../linker/ATSAMD21G18A.ld)
set(CHIP_SVD_FILE ${CMAKE_CURRENT_LIST_DIR}/../../svd/ATSAMD21G18A.svd)

# What src/chip/Variant.hpp reads. A define and not __has_include(<peripherals/USB_DEVICE.hpp>): a build tree that
# changed its part keeps the other part's generated headers.
list(APPEND CHIP_OPTIONS -DKVASIR_CHIP_ATSAMD21G18A=1)
