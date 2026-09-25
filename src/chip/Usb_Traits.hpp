#pragma once

// What the SAM USB backend (atsam_common/usb/Backend.hpp) has to know about this chip: where the
// registers are and what this SVD calls them, the pins, the interrupt, the clock gates and the
// pad calibration. Only the parts with a USB have it (Variant.hpp).

#include "GCLK.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "PM.hpp"
#include "Variant.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/USB_DEVICE.hpp"

#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace USB { namespace Sam {
    struct Traits {
        using Regs = Kvasir::Peripheral::USB_DEVICE::Registers<>;

        // The pack's SVD nests the endpoint registers in a DEVICE_ENDPOINT<N> cluster; an older
        // one lists them as registers with an index, EPCFG<N> (svd/README.md). The backend sees
        // neither spelling.
        template<unsigned N>
        struct Endpoint {
            using Group       = typename Regs::template DEVICE_ENDPOINT<N>;
            using EPCFG       = typename Group::EPCFG;
            using EPSTATUSCLR = typename Group::EPSTATUSCLR;
            using EPSTATUSSET = typename Group::EPSTATUSSET;
            using EPSTATUS    = typename Group::EPSTATUS;
            using EPINTFLAG   = typename Group::EPINTFLAG;
            using EPINTENCLR  = typename Group::EPINTENCLR;
            using EPINTENSET  = typename Group::EPINTENSET;
        };

        // Endpoint numbers 0 to 7 (datasheet DS40001882, 32.1 "Overview": "The USB device mode
        // supports 8 endpoint addresses").
        static constexpr std::size_t EndpointCount = 8;

        static constexpr auto InterruptIndexes = brigand::list<decltype(Kvasir::Interrupt::usb)>{};

        // CLK_USB_AHB and CLK_USB_APB (PM.hpp has the two mask bits).
        static constexpr auto powerClockEnable
          = list(typename Kvasir::PM::enable<Regs::baseAddr>::action{});

        // GCLK_USB has to be 48 MHz +-0.25 % (32.5.3): which generator feeds it is the clock
        // settings' business, this is the channel they route it to.
        static constexpr auto gclkChannel = Kvasir::GCLK::Peripheral::usb;

        // USB/DM is PA24, USB/DP is PA25, both peripheral function G (table 7-1, column COM).
        static constexpr auto pinConfig = list(
          action(Kvasir::Io::Action::PinFunction<6>{}, Kvasir::Register::PinLocation<0, 24>{}),
          action(Kvasir::Io::Action::PinFunction<6>{}, Kvasir::Register::PinLocation<0, 25>{}));

        struct PadCalibration {
            std::uint8_t transn{};
            std::uint8_t transp{};
            std::uint8_t trim{};
        };

        // The NVM software calibration area, 128 bits from 0x00806020 (table 10-8): USB TRANSN is
        // bits 49:45, USB TRANSP 54:50, USB TRIM 57:55 - all in the second word.
        [[nodiscard]] static constexpr PadCalibration decodePadCalibration(std::uint32_t word1) {
            PadCalibration cal{.transn = static_cast<std::uint8_t>((word1 >> 13U) & 0x1FU),
                               .transp = static_cast<std::uint8_t>((word1 >> 18U) & 0x1FU),
                               .trim   = static_cast<std::uint8_t>((word1 >> 23U) & 0x07U)};
            // A field that reads all ones was never programmed. The datasheet does not say what
            // to load then. 5 / 29 / 3 is, as far as remembered, what Microchip's ASF USB driver
            // loads in that case - NOT checked against a source: the datasheets folder has none.
            if(cal.transn == 0x1FU) { cal.transn = 5; }
            if(cal.transp == 0x1FU) { cal.transp = 29; }
            if(cal.trim == 0x07U) { cal.trim = 3; }
            return cal;
        }

        // For the backend's self-test: CLK_USB_AHB and CLK_USB_APB both on, and the area read
        // as something a factory wrote (a field of all ones never was).
        [[nodiscard]] static bool busClocksEnabled() {
            return Kvasir::PM::isEnabled(Regs::baseAddr);
        }

        [[nodiscard]] static bool padCalibrationProgrammed() {
            auto const* const area = reinterpret_cast<std::uint32_t const volatile*>(
              std::uintptr_t{0x00806020});   // NOLINT(performance-no-int-to-ptr)
            std::uint32_t const word1 = area[1];
            return ((word1 >> 13U) & 0x1FU) != 0x1FU && ((word1 >> 18U) & 0x1FU) != 0x1FU
                && ((word1 >> 23U) & 0x07U) != 0x07U;
        }

        [[nodiscard]] static PadCalibration padCalibration() {
            auto const* const area = reinterpret_cast<std::uint32_t const volatile*>(
              std::uintptr_t{0x00806020});   // NOLINT(performance-no-int-to-ptr)
            return decodePadCalibration(area[1]);
        }
    };

    static_assert(Traits::decodePadCalibration((7U << 13U) | (20U << 18U) | (2U << 23U)).transn == 7
                    && Traits::decodePadCalibration((7U << 13U) | (20U << 18U) | (2U << 23U)).transp
                         == 20
                    && Traits::decodePadCalibration((7U << 13U) | (20U << 18U) | (2U << 23U)).trim
                         == 2,
                  "bits 49:45, 54:50 and 57:55 of the area are bits 17:13, 22:18 and 25:23 of its "
                  "second word");
    static_assert(Traits::decodePadCalibration(0xFFFF'FFFF).transn == 5,
                  "an unprogrammed area gets the typical values");
}}}   // namespace Kvasir::USB::Sam
