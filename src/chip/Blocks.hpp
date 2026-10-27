#pragma once

// The SAM D21's clock gates (chip_atsam_common ClockGate.hpp, kvasir/Hal/Block.hpp): every block with a register set
// and a PM mask bit, with the field that clocks it (DS40001882L 16.8.7 AHBMASK, 16.8.8 APBAMASK, 16.8.9 APBBMASK,
// 16.8.10 APBCMASK). Rows marked open at boot have their bit set out of reset - AHBMASK, APBAMASK and APBBMASK reset
// to 0x7F, APBCMASK to 0x00010000 (ADC); Table 16-1 "Peripheral Clock Default State", md l.5755-5784 - and claim
// nothing; a Startup-phase write into any other block needs a listed PM::enable of it. "A module may be connected to
// several clock domains (for instance, AHB and APB), in which case it will have several mask bits" (16.6.2.6, md
// l.5787): such a block has a row per bit - the DSU (CLK_DSU_APB and CLK_DSU_AHB, 13.5.3 md l.2899), the NVMCTRL
// ("two synchronous clocks", 22.5.2 md l.15655), the DMAC (CLK_DMAC_AHB and CLK_DMAC_APB, 20.5.3 md l.12397-12409)
// and the USB. The AHB bridges (HPB0-2) are no blocks; the PTC has no register set in the SVDs.
//
// The SERCOM rows also name what the shared drivers read through Sam::Block<Regs>: the interrupt line (Table 12-1
// "Peripherals Configuration Summary", IRQ Line, md l.2775-2780; one line per SERCOM) and the DMAC trigger sources
// (CHCTRLB.TRIGSRC, 20.8.19 md l.14204-14235: SERCOMn RX = 1 + 2n, TX = 2 + 2n), as the generated enumerators.

#include "Dmac_Traits.hpp"
#include "Interrupt.hpp"
#include "atsam_common/ClockGate.hpp"
#include "peripherals/AC.hpp"
#include "peripherals/ADC.hpp"
#include "peripherals/DAC.hpp"
#include "peripherals/DMAC.hpp"
#include "peripherals/DSU.hpp"
#include "peripherals/EIC.hpp"
#include "peripherals/EVSYS.hpp"
#include "peripherals/GCLK.hpp"
#include "peripherals/HMATRIX.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "peripherals/PAC0.hpp"
#include "peripherals/PAC1.hpp"
#include "peripherals/PAC2.hpp"
#include "peripherals/PM.hpp"
#include "peripherals/PORT.hpp"
#include "peripherals/RTC_MODE0.hpp"
#include "peripherals/SERCOM_I2CM.hpp"
#include "peripherals/SYSCTRL.hpp"
#include "peripherals/TCC.hpp"
#include "peripherals/TC_COUNT16.hpp"
#include "peripherals/WDT.hpp"
#ifdef KVASIR_CHIP_ATSAMD21G18A
    #include "peripherals/I2S.hpp"
    #include "peripherals/USB_DEVICE.hpp"
#endif

namespace Kvasir { namespace Chip { namespace Blocks {
    namespace P = Peripheral;
    using M     = P::PM::Registers<>;
    using I     = ::Kvasir::Interrupt;
    using T     = ::Kvasir::DMAC::Traits::DmacTraits::TriggerSource;
    template<typename Regs, auto const& Field, bool OpenAtBoot = false>
    using Row = ::Kvasir::Sam::GateRow<M::baseAddr, Regs::baseAddr, Field, OpenAtBoot>;
    // a SERCOM: its gate, its interrupt line(s) and its DMAC trigger sources
    template<unsigned N, auto const& Field, typename Lines, T Rx, T Tx>
    using SercomRow = ::Kvasir::Sam::
      GateRow<M::baseAddr, P::SERCOM_I2CM::Registers<N>::baseAddr, Field, false, Lines, Rx, Tx>;
    template<typename... Lines>
    using Irqs = brigand::list<Lines...>;

    // clang-format off
    using Table = ::Kvasir::Hal::BlockTable<
        // two gates each: AHBMASK bits 3, 4, 5 and APBBMASK bits 1, 2, 4
        Row<P::DSU::Registers<>, M::AHBMASK::dsu_, true>,
        Row<P::DSU::Registers<>, M::APBBMASK::dsu_, true>,
        Row<P::NVMCTRL::Registers<>, M::AHBMASK::nvmctrl_, true>,
        Row<P::NVMCTRL::Registers<>, M::APBBMASK::nvmctrl_, true>,
        Row<P::DMAC::Registers<>, M::AHBMASK::dmac_, true>,
        Row<P::DMAC::Registers<>, M::APBBMASK::dmac_, true>,
        Row<P::PAC0::Registers<>, M::APBAMASK::pac0_, true>,
        Row<P::PM::Registers<>, M::APBAMASK::pm_, true>,
        Row<P::SYSCTRL::Registers<>, M::APBAMASK::sysctrl_, true>,
        Row<P::GCLK::Registers<>, M::APBAMASK::gclk_, true>,
        Row<P::WDT::Registers<>, M::APBAMASK::wdt_, true>,
        Row<P::RTC_MODE0::Registers<>, M::APBAMASK::rtc_, true>,
        Row<P::EIC::Registers<>, M::APBAMASK::eic_, true>,
        Row<P::PAC1::Registers<>, M::APBBMASK::pac1_, true>,
        Row<P::PORT::Registers<>, M::APBBMASK::port_, true>,
        // CLK_HMATRIX_APB (Table 16-1): APBBMASK bit 6, set in the reset value 0x7F; 16.8.9's bit table names bits 5:0
        Row<P::HMATRIX::Registers<>, M::APBBMASK::hmatrix_, true>,
        // CLK_PAC2_APB is off out of reset (11.6.1, md l.2096)
        Row<P::PAC2::Registers<>, M::APBCMASK::pac2_>,
        Row<P::EVSYS::Registers<>, M::APBCMASK::evsys_>,
        SercomRow<0, M::APBCMASK::sercom0_, Irqs<decltype(I::sercom0)>, T::sercom0rx, T::sercom0tx>,
        SercomRow<1, M::APBCMASK::sercom1_, Irqs<decltype(I::sercom1)>, T::sercom1rx, T::sercom1tx>,
        SercomRow<2, M::APBCMASK::sercom2_, Irqs<decltype(I::sercom2)>, T::sercom2rx, T::sercom2tx>,
        SercomRow<3, M::APBCMASK::sercom3_, Irqs<decltype(I::sercom3)>, T::sercom3rx, T::sercom3tx>,
        SercomRow<4, M::APBCMASK::sercom4_, Irqs<decltype(I::sercom4)>, T::sercom4rx, T::sercom4tx>,
        SercomRow<5, M::APBCMASK::sercom5_, Irqs<decltype(I::sercom5)>, T::sercom5rx, T::sercom5tx>,
        Row<P::TCC::Registers<0>, M::APBCMASK::tcc0_>,
        Row<P::TCC::Registers<1>, M::APBCMASK::tcc1_>,
        Row<P::TCC::Registers<2>, M::APBCMASK::tcc2_>,
        // TC3-TC5 (APBCMASK bits 11-13, 16.8.10). TC6/TC7 (bits 15:14) are on the J, GxL and J17D parts (five TCs,
        // DS40001882L Table 2-1 "5-2", md l.670-686, read on the PDF page 13-14) and on the 45-ball WLCSP G17A/G18A
        // (its note 1), not on the TQFP/QFN D21G (Table 7-1 note 4, md l.1105); TCC3 (bit 24) only on the x17D and
        // x17L parts (Table 7-1 note 8, Table 7-2 note 8, md l.1121, l.1191). So the G17L rows below have all three.
        Row<P::TC_COUNT16::Registers<3>, M::APBCMASK::tc3_>,
        Row<P::TC_COUNT16::Registers<4>, M::APBCMASK::tc4_>,
        Row<P::TC_COUNT16::Registers<5>, M::APBCMASK::tc5_>,
        Row<P::ADC::Registers<>, M::APBCMASK::adc_, true>,
        Row<P::AC::Registers<0>, M::APBCMASK::ac_>,
        Row<P::DAC::Registers<>, M::APBCMASK::dac_>,
#ifdef KVASIR_CHIP_ATSAMD21G18A
        // Two masks gate the USB: the AHB clock (AHBMASK bit 6, 16.8.7) and the APB one (APBBMASK bit 5, 16.8.9).
        Row<P::USB_DEVICE::Registers<>, M::AHBMASK::usb_, true>,
        Row<P::USB_DEVICE::Registers<>, M::APBBMASK::usb_, true>,
        Row<P::I2S::Registers<>, M::APBCMASK::i2s_>
#else
        Row<P::AC::Registers<1>, M::APBCMASK::ac1_>,
        Row<P::TC_COUNT16::Registers<6>, M::APBCMASK::tc6_>,
        Row<P::TC_COUNT16::Registers<7>, M::APBCMASK::tc7_>,
        Row<P::TCC::Registers<3>, M::APBCMASK::tcc3_>
#endif
        >;
    // clang-format on
}}}   // namespace Kvasir::Chip::Blocks

template<>
struct Kvasir::Hal::BlockGates<Kvasir::Tag::User> {
    using type = Kvasir::Chip::Blocks::Table;
};
