# ATSAMD21G17L: 128 KB flash, 16 KB RAM, a 4 KB RWW section used as EEPROM. No USB.
set(chip_mpu ATSAMD21G17L)

set(chip_linker_file ${CMAKE_CURRENT_LIST_DIR}/../../linker/chip.ld)
set(chip_svd_file ${CMAKE_CURRENT_LIST_DIR}/../../chip.svd)

# What src/chip/Variant.hpp reads: the part, the series and the family, for every TU of the image (kvasir_chip
# DEFINITIONS). Every part has its define, the default one too: a TU without one is an #error there, not silently this
# part.
set(chip_definitions KVASIR_CHIP_ATSAMD21G17L=1 KVASIR_CHIP_ATSAMD21=1 KVASIR_CHIP_FAMILY_SAM=1)
