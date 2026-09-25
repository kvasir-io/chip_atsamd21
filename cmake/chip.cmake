include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)

# Which SAM D21 this firmware is for. A project sets it before include(kvasir.cmake), or passes it as
# -DKVASIR_ATSAMD21_MPU=...; without it the part is the ATSAMD21G17L this package started with.
if(NOT KVASIR_ATSAMD21_MPU)
    set(KVASIR_ATSAMD21_MPU ATSAMD21G17L)
endif()
if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/variants/${KVASIR_ATSAMD21_MPU}.cmake)
    file(
        GLOB _known
        RELATIVE ${CMAKE_CURRENT_LIST_DIR}/variants
        ${CMAKE_CURRENT_LIST_DIR}/variants/*.cmake)
    list(TRANSFORM _known REPLACE "\\.cmake$" "")
    message(FATAL_ERROR "KVASIR_ATSAMD21_MPU is '${KVASIR_ATSAMD21_MPU}'; chip_atsamd21 knows: ${_known}")
endif()
include(${CMAKE_CURRENT_LIST_DIR}/variants/${KVASIR_ATSAMD21_MPU}.cmake)

set(TARGET_UF2_CODE 0x68ED2B88)

svd_convert(peripherals SVD_FILE ${CHIP_SVD_FILE} OUTPUT_DIRECTORY peripherals)

# kvasir_devices: chip.hpp includes its drivers unconditionally (SamPushButton/SamRotaryEncoder ->
# kvasir/Devices/PushButton.hpp, RotaryEncoder.hpp; Sercom_I2CQueued.hpp -> kvasir/Devices/I2C/LineRecovery.hpp; the USB
# backend), so every image needs it. Found like CHIP_ROOT (KVASIR_DEVICES_ROOT: variable, environment, else next to the
# SDK); the SDK adds it after project() unless the firmware has it already.
kvasir_resolve_root(KVASIR_DEVICES_ROOT kvasir_devices)
kvasir_add_package(${KVASIR_DEVICES_ROOT} kvasir_devices kvasir_devices)
target_link_libraries(peripherals INTERFACE kvasir::devices)
