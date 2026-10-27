#pragma once
// The SAM D21's NVM software calibration area for atsam_common/NvmCalibration.hpp: 128 bits at 0x806020, written in
// production, "can not be written" (DS40001882L 10.3.2, Table 10-8). Every field is in the first two words; bits 2:0,
// 14:3, 26:15, 73:64 and 127:74 are reserved. ADC LINEARITY straddles the words (word 0 bits 31:27, word 1 bits 2:0).

#include "atsam_common/NvmCalibration.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace Nvm {
    struct Calibration {
        std::uint8_t adcLinearity{};   // 34:27, to ADC CALIB.LINEARITY_CAL
        std::uint8_t adcBiasCal{};     // 37:35, to ADC CALIB.BIAS_CAL
        std::uint8_t osc32kCal{};      // 44:38, to SYSCTRL OSC32K.CALIB
        std::uint8_t usbTransn{};      // 49:45, to USB PADCAL.TRANSN
        std::uint8_t usbTransp{};      // 54:50, to USB PADCAL.TRANSP
        std::uint8_t usbTrim{};        // 57:55, to USB PADCAL.TRIM
        std::uint8_t dfllCoarse{};     // 63:58, to SYSCTRL DFLLVAL.COARSE

        constexpr bool operator==(Calibration const&) const = default;
    };

    struct CalibrationLayout {
        static constexpr std::uintptr_t address = 0x0080'6020;
        static constexpr std::size_t    words   = 2;
        using Fields                            = Calibration;

        [[nodiscard]] static constexpr Fields decode(std::array<std::uint32_t,
                                                                words> const& w) {
            return {.adcLinearity
                    = static_cast<std::uint8_t>((w[0] >> 27U) | ((w[1] & 0x7U) << 5U)),
                    .adcBiasCal = static_cast<std::uint8_t>((w[1] >> 3U) & 0x7U),
                    .osc32kCal  = static_cast<std::uint8_t>((w[1] >> 6U) & 0x7FU),
                    .usbTransn  = static_cast<std::uint8_t>((w[1] >> 13U) & 0x1FU),
                    .usbTransp  = static_cast<std::uint8_t>((w[1] >> 18U) & 0x1FU),
                    .usbTrim    = static_cast<std::uint8_t>((w[1] >> 23U) & 0x7U),
                    .dfllCoarse = static_cast<std::uint8_t>((w[1] >> 26U) & 0x3FU)};
        }
    };

    // one field at a time, all its bits set: position and width (Table 10-8)
    static_assert(CalibrationLayout::decode({0xF800'0000U,
                                             0x7U})
                    == Calibration{.adcLinearity = 0xFF},
                  "ADC LINEARITY 34:27 straddles the words");
    static_assert(CalibrationLayout::decode({0,
                                             0x7U << 3U})
                    == Calibration{.adcBiasCal = 0x7},
                  "ADC BIASCAL 37:35");
    static_assert(CalibrationLayout::decode({0,
                                             0x7FU << 6U})
                    == Calibration{.osc32kCal = 0x7F},
                  "OSC32K CAL 44:38");
    static_assert(CalibrationLayout::decode({0,
                                             0x1FU << 13U})
                    == Calibration{.usbTransn = 0x1F},
                  "USB TRANSN 49:45");
    static_assert(CalibrationLayout::decode({0,
                                             0x1FU << 18U})
                    == Calibration{.usbTransp = 0x1F},
                  "USB TRANSP 54:50");
    static_assert(CalibrationLayout::decode({0,
                                             0x7U << 23U})
                    == Calibration{.usbTrim = 0x7},
                  "USB TRIM 57:55");
    static_assert(CalibrationLayout::decode({0,
                                             0x3FU << 26U})
                    == Calibration{.dfllCoarse = 0x3F},
                  "DFLL48M COARSE CAL 63:58");
    static_assert(CalibrationLayout::decode({0x07FF'FFFFU,
                                             0})
                    == Calibration{},
                  "word 0 bits 26:0 are reserved");
    static_assert(CalibrationLayoutTrait<CalibrationLayout>);
}}   // namespace Kvasir::Nvm
