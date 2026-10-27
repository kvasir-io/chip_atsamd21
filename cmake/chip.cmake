include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)

# Which SAM D21 this firmware is for. A project sets it before include(kvasir.cmake), or passes it as
# -DKVASIR_ATSAMD21_MPU=...; without it the part is the ATSAMD21G17L this package started with. cmake/variants/ holds
# what differs per part; the ATSAMD21G18A's SVD is made from chip.svd at configure time.
kvasir_chip_variant(
    _variant
    VARIABLE
    KVASIR_ATSAMD21_MPU
    DEFAULT
    ATSAMD21G17L
    DIR
    ${CMAKE_CURRENT_LIST_DIR}/variants
    PACKAGE
    chip_atsamd21
    GENERATED_SVD
    ATSAMD21G18A
    ${CMAKE_CURRENT_LIST_DIR}/../scripts/make_g18a_svd.py
    ${CMAKE_CURRENT_LIST_DIR}/../chip.svd
    ${CMAKE_CURRENT_LIST_DIR}/../svd/samd21a_pack_usb_i2s.svd
    SVD_VARIABLE
    chip_svd_file)
include(${_variant})

# Kvasir's write semantics (svd_convert's defaults): a new unclassified write-only field stops the build
kvasir_chip_svd(peripherals ${chip_svd_file})

# the chip's part of the contract (Kvasir_SDK cmake/kvasir_chip.cmake), with the part's values from its variant file
# DEVICES: chip/drivers.hpp builds on kvasir_devices, which the SDK then adds after project()
kvasir_chip(
    DEVICES
    MPU
    ${chip_mpu}
    UF2_FAMILY
    0x68ED2B88
    LINKER_FILE
    ${chip_linker_file}
    INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/../src
    REGISTERS
    peripherals
    DEFINITIONS
    ${chip_definitions})
