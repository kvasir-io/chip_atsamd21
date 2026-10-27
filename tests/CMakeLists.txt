cmake_minimum_required(VERSION 3.28)

# Host tests of this chip's clock limits (src/chip/ClockLimits.hpp) through chip_atsam_common's ClockSolver.hpp, from
# the src/chip/atsam_common this repository records - the solver itself is tested in chip_atsam_common. Needs
# Kvasir_SDK's Util/Prescaler.hpp (KVASIR_ROOT: the checkout next to this one, else fetched):
#
# cmake -S tests -B tests/build -G Ninja && cmake --build tests/build && ctest --test-dir tests/build
#
# Most checks are static_asserts: building is the test.
#
# The must_fail_* tests compile must_fail.cpp with -DMUST_FAIL=n and look for the message, numbers included, in the
# compiler's output.
project(chip_atsamd21_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# Kvasir_SDK: KVASIR_ROOT, else the checkout next to this repository, else fetched; its submodules and the other
# packages: kvasir_locate (Kvasir_SDK cmake/kvasir_locate.cmake)
if(NOT KVASIR_ROOT AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK/cmake/kvasir_locate.cmake")
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK" ABSOLUTE)
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
kvasir_locate(SDK_SUBMODULES svd_converter)
get_filename_component(_chip_src "${CMAKE_CURRENT_LIST_DIR}/../src/chip" ABSOLUTE)
# kvasir::base: the SDK's src/ (Util/Prescaler.hpp, Register/, ...)
include(${KVASIR_ROOT}/cmake/kvasir_sdk_targets.cmake)

add_executable(clock_limits clock_limits_test.cpp)
target_include_directories(clock_limits PRIVATE ${_chip_src})
target_link_libraries(clock_limits PRIVATE kvasir::base)
target_compile_options(clock_limits PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_limits COMMAND clock_limits)

add_executable(reset_cause reset_cause_test.cpp)
target_include_directories(reset_cause PRIVATE ${_chip_src})
target_link_libraries(reset_cause PRIVATE kvasir::base)
target_compile_options(reset_cause PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME reset_cause COMMAND reset_cause)

add_executable(calibration calibration_test.cpp)
target_include_directories(calibration PRIVATE ${_chip_src})
target_link_libraries(calibration PRIVATE kvasir::base)
target_compile_options(calibration PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME calibration COMMAND calibration)

add_executable(fuse_traits fuse_traits_test.cpp)
target_include_directories(fuse_traits PRIVATE ${_chip_src})
target_link_libraries(fuse_traits PRIVATE kvasir::base)
target_compile_options(fuse_traits PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME fuse_traits COMMAND fuse_traits)

set(_must_fail_messages
    "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 \\(SAM D21 Table 37-42\\), RWS is 1"
    "flash wait states: no count for 49000000 Hz at 2.7 V and above in SAM D21 Table 37-42"
    "FDPLL reference: wanted 32000..2000000 Hz, got 31007.8 Hz \\(SAM D21 FDPLL96M, Tables 37-58/59/60\\)"
    "SAM D21 FDPLL96M, Tables 37-58/59/60: no setting makes 200000000 Hz from 8000000 Hz \\(integer mode, 0 ppm\\)"
    "DFLL48M reference: wanted 732..33000 Hz, got 40000.0 Hz \\(SAM D21 Table 37-54 f_REF\\)"
    "SAM D21 at 105 C: DS40001882L chapter 38 has no wait-state table"
    "DFLL48M closed loop: wanted 48000000 Hz, got 48005120.0 Hz \\(\\+107 ppm\\), allowed 100 ppm")
set(_n 0)
foreach(_message IN LISTS _must_fail_messages)
    math(EXPR _n "${_n} + 1")
    add_test(
        NAME must_fail_${_n}
        COMMAND
            ${CMAKE_CXX_COMPILER} -std=c++26 -fsyntax-only
            -I$<TARGET_PROPERTY:kvasir_base,INTERFACE_INCLUDE_DIRECTORIES> -I${_chip_src} -DMUST_FAIL=${_n}
            ${CMAKE_CURRENT_LIST_DIR}/must_fail.cpp)
    set_tests_properties(must_fail_${_n} PROPERTIES PASS_REGULAR_EXPRESSION "${_message}")
endforeach()

# The write-only guard: every write-only field of this chip's SVD(s) is classified (oneToSet, a key, or <!-- Kvasir:
# write-only accepted -->), so svd_converter --write-only-guard=error must exit 0. Needs the SDK's svd_converter (the
# sibling or fetched SDK's submodule); left out without it, except in CI. chip.svd (the G17L) and the G18A's SVD, which
# this tree generates from it the way a firmware tree does (the SDK's cmake/chip_package.cmake).
include(${KVASIR_ROOT}/cmake/chip_package.cmake)
kvasir_chip_generate_svd(
    _g18a_svd
    PART
    ATSAMD21G18A
    SCRIPT
    ${CMAKE_CURRENT_LIST_DIR}/../scripts/make_g18a_svd.py
    INPUTS
    ${CMAKE_CURRENT_LIST_DIR}/../chip.svd
    ${CMAKE_CURRENT_LIST_DIR}/../svd/samd21a_pack_usb_i2s.svd)
if(EXISTS ${KVASIR_ROOT}/svd_converter/CMakeLists.txt)
    add_subdirectory(${KVASIR_ROOT}/svd_converter svd_converter)
    foreach(svd ${CMAKE_CURRENT_LIST_DIR}/../chip.svd ${_g18a_svd})
        get_filename_component(_svd_name ${svd} NAME)
        string(MAKE_C_IDENTIFIER ${_svd_name} _svd_name)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/write_only_guard/${_svd_name})
        add_test(NAME svd_write_only_guard_${_svd_name}
                 COMMAND svd_converter --write-only-guard=error --write-only-registers=derived ${svd}
                         ${CMAKE_CURRENT_BINARY_DIR}/write_only_guard/${_svd_name} json)
    endforeach()
else()
    # in CI the guard must run: a missing svd_converter there is a broken fetch, not a reason to pass
    if(DEFINED ENV{CI})
        message(FATAL_ERROR "svd_write_only_guard: no ${KVASIR_ROOT}/svd_converter in CI")
    endif()
    message(STATUS "svd_write_only_guard left out: no ${KVASIR_ROOT}/svd_converter")
endif()
