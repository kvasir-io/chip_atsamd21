#pragma once

// Which SAM D21 this firmware is built for: cmake/chip.cmake's KVASIR_ATSAMD21_MPU picks the SVD,
// the linker script and, for every part but the default one, a define that is read here. The
// headers of this package differ between the parts only where the silicon does, and ask this.
namespace Kvasir { namespace Chip {
    enum class Variant { atsamd21g17l, atsamd21g18a };

#if defined(KVASIR_CHIP_ATSAMD21G18A)
    inline constexpr Variant variant = Variant::atsamd21g18a;
#else
    inline constexpr Variant variant = Variant::atsamd21g17l;
#endif

    // The L parts trade the USB and the I2S for a second analog comparator, TC6/TC7 and TCC3
    // (SAM D21 datasheet DS40001882, "Configuration Summary").
    inline constexpr bool hasUsb = variant == Variant::atsamd21g18a;
}}   // namespace Kvasir::Chip

// The generated peripheral headers have to be the selected part's: a build tree that changed its
// part without being wiped keeps the old ones.
#if defined(KVASIR_CHIP_ATSAMD21G18A)
    #if !__has_include("peripherals/USB_DEVICE.hpp")
        #error \
          "KVASIR_CHIP_ATSAMD21G18A is set, but the generated peripherals have no USB: wipe the build tree"
    #endif
#endif
