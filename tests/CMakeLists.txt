cmake_minimum_required(VERSION 3.28)

# Host tests of this chip's clock limits (src/chip/ClockLimits.hpp) through chip_atsam_common's ClockSolver.hpp, from
# the src/chip/atsam_common this repository records - the solver itself is tested in chip_atsam_common. Needs
# Kvasir_SDK's Util/Prescaler.hpp (KVASIR_ROOT, default: the checkout next to this one):
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

if(NOT KVASIR_ROOT)
    get_filename_component(KVASIR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../Kvasir_SDK" ABSOLUTE)
endif()
get_filename_component(_chip_src "${CMAKE_CURRENT_LIST_DIR}/../src/chip" ABSOLUTE)

add_executable(clock_limits clock_limits_test.cpp)
target_include_directories(clock_limits PRIVATE ${KVASIR_ROOT}/src ${_chip_src})
target_compile_options(clock_limits PRIVATE -Wall -Wextra -Wpedantic -Werror)
add_test(NAME clock_limits COMMAND clock_limits)

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
    add_test(NAME must_fail_${_n} COMMAND ${CMAKE_CXX_COMPILER} -std=c++26 -fsyntax-only -I${KVASIR_ROOT}/src
                                          -I${_chip_src} -DMUST_FAIL=${_n} ${CMAKE_CURRENT_LIST_DIR}/must_fail.cpp)
    set_tests_properties(must_fail_${_n} PROPERTIES PASS_REGULAR_EXPRESSION "${_message}")
endforeach()
