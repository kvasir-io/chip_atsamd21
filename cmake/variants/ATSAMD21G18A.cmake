# ATSAMD21G18A: 256 KB flash, 32 KB RAM, no RWW section (so no EEPROM region), USB. (SAM D21 datasheet DS40001882,
# "Configuration Summary" and table 10-3 "SAM D21 Flash Memory Parameters".)
set(chip_mpu ATSAMD21G18A)

set(chip_linker_file ${CMAKE_CURRENT_LIST_DIR}/../../linker/ATSAMD21G18A.ld)
# its SVD is chip.svd without the L-only peripherals and with the pack's USB and I2S, made at configure (never checked
# in): generated, see chip.cmake

# What src/chip/Variant.hpp reads (the part, the series, the family). A define and not
# __has_include(<peripherals/USB_DEVICE.hpp>): a build tree that changed its part keeps the other part's generated
# headers.
set(chip_definitions KVASIR_CHIP_ATSAMD21G18A=1 KVASIR_CHIP_ATSAMD21=1 KVASIR_CHIP_FAMILY_SAM=1)
