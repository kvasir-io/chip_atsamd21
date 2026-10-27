#pragma once

#include <string_view>

// Which SAM D21 this firmware is built for: cmake/chip.cmake's KVASIR_ATSAMD21_MPU picks the SVD,
// the linker script and the part's define (cmake/variants/<part>.cmake: KVASIR_CHIP_<part>, with
// KVASIR_CHIP_ATSAMD21 and KVASIR_CHIP_FAMILY_SAM), which is read here. The headers of this package
// differ between the parts only where the silicon does, and ask this.
namespace Kvasir { namespace Chip {
    enum class Part { atsamd21g17l, atsamd21g18a };

#ifdef KVASIR_CHIP_ATSAMD21G18A
    inline constexpr Part part = Part::atsamd21g18a;
#elifdef KVASIR_CHIP_ATSAMD21G17L
    inline constexpr Part part = Part::atsamd21g17l;
#else
    #error \
      "chip_atsamd21 without a KVASIR_CHIP_ATSAMD21<part> define: not compiled with the package's kvasir_chip DEFINITIONS"
#endif

    // which part this image is for, as text
    inline constexpr std::string_view partName
      = part == Part::atsamd21g18a ? "ATSAMD21G18A" : "ATSAMD21G17L";

    // The L parts trade the USB and the I2S for a second analog comparator, TC6/TC7 and TCC3
    // (SAM D21 datasheet DS40001882, "Configuration Summary").
    inline constexpr bool hasUsb = part == Part::atsamd21g18a;
}}   // namespace Kvasir::Chip
