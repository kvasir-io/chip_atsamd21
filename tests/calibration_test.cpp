// Kvasir::Nvm::calibration() over this chip's layout (src/chip/NvmCalibration_Traits.hpp, SAM D21 DS40001882L 10.3.2
// Table 10-8): the field positions are static_asserts in the header; here the read itself - two word loads at
// 0x806020 and 0x806024 through Kvasir::Memory, in that order, decoded - with the register mock. The mock takes every
// width, as the SDK's seam declares it (Register/Mmio.hpp: Mmio, Words, mmioLoadBytes byte by byte - a mock TU
// instantiates them all), and records the width of each read, so "two words" is checked, not assumed.
#define KVASIR_REGISTER_MOCK 1
#include "NvmCalibration_Traits.hpp"

#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

namespace {
std::vector<std::pair<unsigned, unsigned>> reads;   // (address, width in bytes)
std::uint32_t                              word0    = 0;
std::uint32_t                              word1    = 0;
int                                        failures = 0;

void check(bool        ok,
           char const* what) {
    if(!ok) {
        ++failures;
        std::printf("FAIL: %s\n", what);
    }
}
}   // namespace

namespace Kvasir { namespace Test {
    template<typename T>
    T readAt(unsigned address) {
        reads.emplace_back(address, static_cast<unsigned>(sizeof(T)));
        return static_cast<T>(address == 0x0080'6020U   ? word0
                              : address == 0x0080'6024U ? word1
                                                        : 0xDEAD'BEEFU);
    }

    template<typename T>
    void writeAt(unsigned,
                 T) {
        ++failures;
        std::printf("FAIL: the calibration area is never written\n");
    }
}}   // namespace Kvasir::Test

int main() {
    // a made-up die: every field a different value, ADC LINEARITY 0xA5 across the words (low 5 bits in word 0)
    word0 = (0xA5U & 0x1FU) << 27U;
    word1 = (0xA5U >> 5U) | (6U << 3U) | (0x55U << 6U) | (9U << 13U) | (22U << 18U) | (3U << 23U)
          | (41U << 26U);
    auto const cal = Kvasir::Nvm::calibration();
    check(reads == std::vector<std::pair<unsigned, unsigned>>{{0x0080'6020U, 4U}, {0x0080'6024U, 4U}},
          "two word loads, 0x806020 then 0x806024");
    check(cal
            == Kvasir::Nvm::Calibration{.adcLinearity = 0xA5,
                                        .adcBiasCal   = 6,
                                        .osc32kCal    = 0x55,
                                        .usbTransn    = 9,
                                        .usbTransp    = 22,
                                        .usbTrim      = 3,
                                        .dfllCoarse   = 41},
          "every field of Table 10-8 decoded");

    // an unprogrammed area reads all ones: every field all ones (the users' fallbacks decide)
    reads.clear();
    word0 = word1    = 0xFFFF'FFFFU;
    auto const blank = Kvasir::Nvm::calibration();
    check(blank.adcLinearity == 0xFF && blank.dfllCoarse == 0x3F && blank.usbTrim == 0x7,
          "an erased area");

    if(failures == 0) { std::printf("calibration: ok\n"); }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
