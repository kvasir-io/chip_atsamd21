#pragma once
// The SAM D21's names the shared drivers of atsam_common use, in one place: atsam_common/StartUp.hpp asserts
// Kvasir::Sam::ChipPackage (atsam_common/ChipTraits.hpp lists what each member must give) on it.

#include "Dmac_Traits.hpp"
#include "Interrupt.hpp"
#include "NVMCTRL_Traits.hpp"
#include "Sercom_Traits.hpp"
#include "Serial_Number_Traits.hpp"
#include "Start.hpp"
#include "chip/atsam_common/SercomPads.hpp"
#include "peripherals/SERCOM_SPI.hpp"
#include "peripherals/SERCOM_USART.hpp"

#include <cstdint>

namespace Kvasir { namespace Sam {
    struct Package {
        using Sercom = Kvasir::Sercom::Traits::SercomTraits;
        using Dmac   = Kvasir::DMAC::Traits::DmacTraits;
        using Nvm    = Kvasir::NVMCTRL::Traits;
        using Start  = Kvasir::Startup::Sam::StartTraits;
        template<unsigned Line>
        using EicVector = Kvasir::Interrupt::EicExtIntVector<Line>;
        template<unsigned I>
        using SercomSpi = Kvasir::Peripheral::SERCOM_SPI::Registers<I>;
        template<unsigned I>
        using SercomUsart = Kvasir::Peripheral::SERCOM_USART::Registers<I>;
        using SercomPads  = Kvasir::Sam::Pads::SamD2xC2x;
        static constexpr auto const& serialNumberAddresses
          = Kvasir::SerialNumber::Traits::addresses;
        // The chip differences the shared drivers read (atsam_common/ChipTraits.hpp): one STATUS.SYNCBUSY bit for the
        // EIC (DS40001882L 21.8.2, md l.15327) and the WDT (18.8.7, md l.10287); the WDT counts the GCLK_WDT the
        // ClockSettings route (its Config::clockSpeed); the DMAC's CRC is turned on by CTRL.CRCENABLE (20.8.1
        // md l.13387)
        static constexpr bool          eicSyncBusyInStatus = true;
        static constexpr bool          wdtSyncBusyInStatus = true;
        static constexpr std::uint32_t wdtFixedClockHz     = 0;
        static constexpr bool          dmacHasCrcEnable    = true;
        static constexpr bool          hasCan              = false;
    };
}}   // namespace Kvasir::Sam
