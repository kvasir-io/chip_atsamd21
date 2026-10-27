#pragma once

// What the SAM USB backend (atsam_common/usb/Backend.hpp) has to know about this chip: where the
// registers are and what this SVD calls them, the pins, the interrupt, the clock gates and the
// pad calibration. Only the parts with a USB have it (Variant.hpp).

#include "GCLK.hpp"
#include "Interrupt.hpp"
#include "NvmCalibration_Traits.hpp"
#include "PM.hpp"
#include "Variant.hpp"
#include "atsam_common/Io.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/USB_DEVICE.hpp"

#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace USB { namespace Sam {
    struct Traits {
        using Regs = Kvasir::Peripheral::USB_DEVICE::Registers<>;

        // The pack's SVD nests the endpoint registers in a DEVICE_ENDPOINT<N> cluster
        // (svd/README.md); the backend does not see that spelling.
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
        static constexpr std::size_t endpointCount = 8;

        static constexpr auto interruptIndexes = brigand::list<decltype(Kvasir::Interrupt::usb)>{};

        // CLK_USB_AHB and CLK_USB_APB (PM.hpp has the two mask bits).
        static constexpr auto powerClockEnable
          = list(typename Kvasir::PM::Enable<Regs::baseAddr>::action{});

        // GCLK_USB has to be 48 MHz +-0.25 % (32.5.3): which generator feeds it is the clock
        // settings' business, this is the channel they route it to.
        static constexpr auto gclkChannel = Kvasir::GCLK::Peripheral::usb;

        // USB/DM is PA24, USB/DP is PA25, both peripheral function G (table 7-1, column COM).
        static constexpr auto pinConfig
          = list(action(Kvasir::Io::Action::PinFunction<6>{}, Kvasir::Io::PinLocation<0, 24>{}),
                 action(Kvasir::Io::Action::PinFunction<6>{}, Kvasir::Io::PinLocation<0, 25>{}));

        struct PadCalibration {
            std::uint8_t transn{};
            std::uint8_t transp{};
            std::uint8_t trim{};
        };

        // USB TRANSN / TRANSP / TRIM of the NVM software calibration area (bits 49:45, 54:50, 57:55; DS40001882L
        // Table 10-8), with a fallback for a field that was never programmed.
        [[nodiscard]] static constexpr PadCalibration
        padCalibrationFrom(Kvasir::Nvm::Calibration const& nvm) {
            PadCalibration cal{.transn = nvm.usbTransn,
                               .transp = nvm.usbTransp,
                               .trim   = nvm.usbTrim};
            // A field that reads all ones was never programmed. The datasheet does not say what
            // to load then (10.3.2 lists the fields only; PADCAL resets to 0x0000, 32.8.6).
            // 5 / 29 / 3 are the values Zephyr's SAM0 USB driver loads in that case, from the same
            // NVM bits 45 / 50 / 55 (zephyr drivers/usb/udc/udc_sam0.c sam0_load_padcal, Zephyr
            // 4.5.0-rc1). Microchip's own ASF/Harmony source was not checked.
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
            auto const nvm = Kvasir::Nvm::calibration();
            return nvm.usbTransn != 0x1FU && nvm.usbTransp != 0x1FU && nvm.usbTrim != 0x07U;
        }

        [[nodiscard]] static PadCalibration padCalibration() {
            return padCalibrationFrom(Kvasir::Nvm::calibration());
        }
    };

    static_assert(Traits::padCalibrationFrom({.usbTransn = 7,
                                              .usbTransp = 20,
                                              .usbTrim   = 2})
                        .transn
                      == 7
                    && Traits::padCalibrationFrom({.usbTransn = 7,
                                                   .usbTransp = 20,
                                                   .usbTrim   = 2})
                           .transp
                         == 20
                    && Traits::padCalibrationFrom({.usbTransn = 7,
                                                   .usbTransp = 20,
                                                   .usbTrim   = 2})
                           .trim
                         == 2,
                  "a programmed field is taken as it is");
    static_assert(Traits::padCalibrationFrom(Kvasir::Nvm::CalibrationLayout::decode({0xFFFF'FFFF,
                                                                                     0xFFFF'FFFF}))
                      .transn
                    == 5,
                  "an unprogrammed area gets the typical values");
}}}   // namespace Kvasir::USB::Sam
