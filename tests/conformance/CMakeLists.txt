cmake_minimum_required(VERSION 3.28)

# The chip's conformance tree (Kvasir_SDK tests/conformance/conformance.cmake): this package's headers compiled for the
# target with a firmware's flags, against the generated register headers. Building is the test - the contract rows
# (conformance.cpp), the dependent register writes (instantiate.cpp) and every public header alone. The chip and core
# SVDs convert with the write-only guard on the way.
#
# cmake -S tests/conformance -B tests/conformance/build -G Ninja [-DCOMPILE_TARGET=arm_gcc]
# [-DCONFORMANCE_VARIANT=<part>] && cmake --build tests/conformance/build
#
# Kvasir_SDK and kvasir_devices: the checkouts next to this one, else fetched (FETCHCONTENT_SOURCE_DIR_KVASIR_SDK /
# _KVASIR_DEVICES point a lone checkout at local copies).

set(CONFORMANCE_VARIANT
    ATSAMD21G17L
    CACHE STRING "the part the tree is configured for (KVASIR_ATSAMD21_MPU): ATSAMD21G17L or ATSAMD21G18A")
set(KVASIR_ATSAMD21_MPU ${CONFORMANCE_VARIANT})

get_filename_component(CHIP_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
# Kvasir_SDK: KVASIR_ROOT, else the checkout next to this repository, else fetched; its submodules and the other
# packages: kvasir_locate (Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../Kvasir_SDK" ABSOLUTE)
elseif(NOT KVASIR_ROOT)
    include(FetchContent)
    FetchContent_Declare(
        kvasir_sdk
        GIT_REPOSITORY https://github.com/kvasir-io/Kvasir_SDK.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        GIT_SUBMODULES "" SOURCE_SUBDIR fetch-only)
    FetchContent_MakeAvailable(kvasir_sdk)
    set(KVASIR_ROOT "${kvasir_sdk_SOURCE_DIR}")
endif()
include("${KVASIR_ROOT}/cmake/kvasir_locate.cmake")
kvasir_locate(SDK_SUBMODULES ALL DEVICES)
if(NOT KVASIR_DEVICES_ROOT)
    FetchContent_Declare(
        kvasir_devices
        GIT_REPOSITORY https://github.com/kvasir-io/kvasir_devices.git
        GIT_TAG master
        GIT_SHALLOW TRUE
        SOURCE_SUBDIR none)
    FetchContent_MakeAvailable(kvasir_devices)
    set(KVASIR_DEVICES_ROOT ${kvasir_devices_SOURCE_DIR})
endif()

include(${KVASIR_ROOT}/tests/conformance/conformance.cmake)
kvasir_conformance_begin()

# a version: main_tu.cpp lists Kvasir::Boot::Report<>, whose boot line names the project's
project(
    chip_atsamd21_conformance
    VERSION 0.0.0
    LANGUAGES CXX C ASM)

include(${KVASIR_ROOT}/cmake/util.cmake)

kvasir_conformance_project(
    NAME
    chip_atsamd21_conformance
    SOURCES
    conformance.cpp
    INSTANTIATE
    instantiate.cpp
    CONFIG_HEADER
    kvasir_config.hpp
    HEADERS
    ${CHIP_ROOT}/src/chip/*.hpp
    ${CHIP_ROOT}/src/chip/atsam_common/*.hpp
    NOT_ALONE
    not_alone.txt
    MAIN_TU
    main_tu.cpp)

# SAM pin ownership in a Startup list (chip_atsam_common Io.hpp: a pin's PINCFG.PMUXEN write in initStepPinConfig
# provides the pin): pins_must_fail.cpp with PINS_FAIL=0 compiles in `all`, each case below must fail naming the pin
# (ctest in this tree, both compilers).
kvasir_conformance_must_fail(
    NAME
    pins_must_fail
    PROJECT
    chip_atsamd21_conformance
    SOURCE
    pins_must_fail.cpp
    DEFINE
    PINS_FAIL
    CASES
    "1|provided [(]configured, owned[)] by two peripherals: Io pin 0[.]17"
    "2|provided [(]configured, owned[)] by two peripherals: Io pin 0[.]8")
