#pragma once

#include "ClockLimits.hpp"
#include "WaitBounds.hpp"
#include "atsam_common/WaitBounds.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/GCLK.hpp"
#include "peripherals/PM.hpp"
#include "peripherals/SYSCTRL.hpp"

#include <cstdint>

namespace Kvasir { namespace GCLK {
    enum class GeneratorSource : unsigned {
        xosc      = 0,   //XOSC oscillator output
        gclkin    = 1,   //Generator input pad
        gclkgen1  = 2,   //Generic clock generator 1 output
        osculp32k = 3,   //OSCULP32K oscillator output
        osc32k    = 4,   //OSC32K oscillator output
        xosc32k   = 5,   //XOSC32K oscillator output
        osc8m     = 6,   //OSC8M oscillator output
        dfll48m   = 7,   //DFLL48M output
        fdpll     = 8,   //FDPLL output
    };

    enum class Peripheral : std::uint16_t {
        dfll48   = 0,   //DFLL48
        fdpll    = 1,   //FDPLL
        fdpll32k = 2,   //FDPLL32K
        wdt      = 3,   //WDT
        rtc      = 4,   //RTC
        eic      = 5,   //EIC
#if defined(KVASIR_CHIP_ATSAMD21G18A)
        usb = 6,   //USB (SAM D21 datasheet, GCLK CLKCTRL.ID: 0x06 GCLK_USB)
#endif
        evsys_0      = 7,    //EVSYS_0
        evsys_1      = 8,    //EVSYS_1
        evsys_2      = 9,    //EVSYS_2
        evsys_3      = 10,   //EVSYS_3
        evsys_4      = 11,   //EVSYS_4
        evsys_5      = 12,   //EVSYS_5
        evsys_6      = 13,   //EVSYS_6
        evsys_7      = 14,   //EVSYS_7
        evsys_8      = 15,   //EVSYS_8
        evsys_9      = 16,   //EVSYS_9
        evsys_10     = 17,   //EVSYS_10
        evsys_11     = 18,   //EVSYS_11
        sercomx_slow = 19,   //SERCOMX_SLOW
        sercom0_core = 20,   //SERCOM0_CORE
        sercom1_core = 21,   //SERCOM1_CORE
        sercom2_core = 22,   //SERCOM2_CORE
        sercom3_core = 23,   //SERCOM3_CORE
        sercom4_core = 24,   //SERCOM4_CORE
        sercom5_core = 25,   //SERCOM5_CORE
        tcc0_tcc1    = 26,   //TCC0_TCC1
        tcc2_tc3     = 27,   //TCC2_TC3
        tc4_tc5      = 28,   //TC4_TC5
        tc6_tc7      = 29,   //TC6_TC7
        adc          = 30,   //ADC
        ac_dig       = 31,   //AC_DIG
        ac_ana       = 32,   //AC_ANA
        dac          = 33,   //DAC
        i2s_0        = 35,   //I2S_0
        i2s_1        = 36,   //I2S_1
    };

    template<unsigned Generator, GeneratorSource Source, unsigned long long Div>
    struct GenericClockGenerator {
        using GC = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL;
        using GD = Kvasir::Peripheral::GCLK::Registers<>::GENDIV;

        static constexpr unsigned maxDiv
          = (Generator == 1 ? (1 << 16) : (Generator == 2 ? (1 << 5) : (1 << 8))) - 1;

        static_assert((Div <= maxDiv) || (std::popcount(Div) == 1),
                      "must be power of 2 or lower then maxDiv");

        static constexpr auto divsel = []() {
            if constexpr(Div > maxDiv) {
                return GC::DIVSELValC::div2;
            } else {
                return GC::DIVSELValC::div1;
            }
        }();
        static constexpr auto div = []() {
            if constexpr(Div > maxDiv) {
                return std::countr_zero(Div) - 1;
            } else {
                return Div;
            }
        }();

        static constexpr bool divisionInTable = Div <= ClockLimits::D21::gclkMaxDivision(Generator);

        [[nodiscard]] static constexpr auto enable() {
            // The data sheet's "Maximum Division Factor" (15.8.5, md l.5407) is what the hardware
            // does: above it the division clamps (/512 for DIVSEL on rev D silicon).
            static_assert(
              divisionInTable,
              "GCLK generator division above the SAM D21's Maximum Division Factor (data "
              "sheet 15.8.5: 512 for generators 0 and 3-8, 64 for 2, 131072 for 1): the "
              "silicon clamps it (measured), so the clock would not be what is written");
            return list(Register::sequencePoint,
                        write(GD::id, Register::value<Generator>()),
                        write(GD::div, Register::value<div>()),
                        Register::sequencePoint,
                        Register::overrideDefaults<typename GC::default_values>::value(
                          write(GC::id, Register::value<Generator>()),
                          set(GC::genen),
                          write(divsel),
                          set(GC::runstdby),
                          write(GC::src,
                                Register::value<typename GC::SRCVal,
                                                static_cast<typename GC::SRCVal>(Source)>())));
        }
    };

    template<unsigned Generator, Peripheral Peripheral>
    struct PeripheralChannelController {
        using CC = Kvasir::Peripheral::GCLK::Registers<>::CLKCTRL;

        [[nodiscard]] static constexpr auto enable() {
            return list(Register::sequencePoint,
                        Register::overrideDefaults<typename CC::default_values>::value(
                          write(CC::id,
                                Register::value<typename CC::IDVal,
                                                static_cast<typename CC::IDVal>(Peripheral)>()),
                          set(CC::clken),
                          write(CC::gen,
                                Register::value<typename CC::GENVal,
                                                static_cast<typename CC::GENVal>(Generator)>())));
        }

        [[nodiscard]] static constexpr auto disable() {
            return list(Register::sequencePoint,
                        Register::overrideDefaults<typename CC::default_values>::value(
                          write(CC::id,
                                Register::value<typename CC::IDVal,
                                                static_cast<typename CC::IDVal>(Peripheral)>()),
                          clear(CC::clken),
                          write(CC::gen,
                                Register::value<typename CC::GENVal,
                                                static_cast<typename CC::GENVal>(Generator)>())));
        }
    };

    // GCLK0's source: GENCTRL is indirect - an 8-bit write of the ID, then a read (15.6.4.1); SRC is bits 12:8
    // (15.8.4).
    inline GeneratorSource mainClockSource() {
        constexpr std::uintptr_t genctrl
          = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL::Addr::value;
        *reinterpret_cast<std::uint8_t volatile*>(genctrl) = 0;   // GENCTRL.ID = 0
        return static_cast<GeneratorSource>(
          (*reinterpret_cast<std::uint32_t const volatile*>(genctrl) >> 8U) & 0x1FU);
    }

    // Whether the chip looks as a reset leaves it: no APBC bus clock but the ADC's (PM.APBCMASK resets to
    // 0x00010000, 16.8.10) and GCLK0 on OSC8M (8.3.1). Any image that used a SERCOM, TC, TCC, DAC or AC, or ran
    // GCLK0 on another source, leaves a difference behind - and a jump does not reset either (SysResetReq and WDT
    // reset all but the RTC, 32 kHz sources, an ALWAYSON WDT and WRTLOCKed GCLKs, Table 16-2).
    inline bool looksLikeAfterReset() {
        auto const apbc = *reinterpret_cast<std::uint32_t const volatile*>(
          Kvasir::Peripheral::PM::Registers<>::APBCMASK::Addr::value);
        return apbc == 0x0001'0000U && mainClockSource() == GeneratorSource::osc8m;
    }

    // GCLK0 (the CPU's clock) onto OSC8M before the clock tree is rewritten. An image a bootloader jumps into, or
    // one that jumps into itself, may find GCLK0 on the DFLL, the FDPLL or the XOSC that ClockTree / a firmware's
    // own tree is about to reprogram - and reference switching the FDPLL means disabling it (17.6.8.5), which
    // stops the CPU if GCLK0 runs on it. After a real reset GCLK0 is on OSC8M already (8.3.1) and this only reads.
    // Zephyr's soc_samd2x.c does the same switch "to allow switching between clocks when using bootloaders".
    // Sequence: GENCTRL is indirect - an 8-bit write of the ID, then a read (15.6.4.1); SRC is bits 12:8 (15.8.4).
    // "Before switching GCLKGEN0 from a clock source A to another clock source B, enable the ONDEMAND feature of
    // the clock source A" (15.6.2.6 Note): bit 7 of XOSC, XOSC32K, OSC32K, DFLLCTRL, DPLLCTRLA (17.8.5/6/7/10,
    // 17.8.17). A DFLL register write needs PCLKSR.DFLLRDY (17.6.14) and is safe while GCLK0 requests the DFLL
    // (errata DS80000760 1.2.1 freezes only a write to a DFLL nobody requests). OSC8M is enabled at reset
    // (17.8.9, ENABLE bit 1); its PRESC is left as found. The switch is on the fly, the generator stays on A
    // until B is ready (15.6.2.6); a GENCTRL write during STATUS.SYNCBUSY stalls the bus and completes (15.6.6).
    // Wait states (NVMCTRL.CTRLB.RWS) stay as they are: OSC8M needs none.
    // OSC8M.ONDEMAND resets to 1 (17.8.9): an on-demand oscillator nobody requests "will be in a disabled state"
    // (17.8.9 bit 7, 14.6) and PCLKSR.OSC8MRDY stays 0 - with GCLK0 on another source nobody requests OSC8M, so
    // waiting for OSC8MRDY before the switch waited for ever (measured: stuck here until a watchdog reset the
    // chip; OSC8M read 0x88060382, PCLKSR 0x5A10). ONDEMAND off for the switch, back on once GCLK0 requests it.
    inline void parkMainClock() {
        using GC          = Kvasir::Peripheral::GCLK::Registers<>;
        using KSR         = Kvasir::Peripheral::SYSCTRL::Registers<>;
        auto const source = mainClockSource();
        if(source == GeneratorSource::osc8m) { return; }
        switch(source) {
        case GeneratorSource::xosc:    apply(set(KSR::XOSC::ondemand)); break;
        case GeneratorSource::xosc32k: apply(set(KSR::XOSC32K::ondemand)); break;
        case GeneratorSource::osc32k:  apply(set(KSR::OSC32K::ondemand)); break;
        case GeneratorSource::dfll48m:
            Kvasir::Register::waitUntil<Kvasir::Chip::DfllSyncBound>(
              Kvasir::Register::isSet(KSR::PCLKSR::dfllrdy));
            apply(set(KSR::DFLLCTRL::ondemand));
            Kvasir::Register::waitUntil<Kvasir::Chip::DfllSyncBound>(
              Kvasir::Register::isSet(KSR::PCLKSR::dfllrdy));
            break;
        case GeneratorSource::fdpll: apply(set(KSR::DPLLCTRLA::ondemand)); break;
        default:                     break;   // GCLKIN, GCLKGEN1, OSCULP32K: no ONDEMAND
        }
        apply(set(KSR::OSC8M::enable), clear(KSR::OSC8M::ondemand));
        Kvasir::Register::waitUntil<Kvasir::Chip::Osc8mReadyBound>(
          Kvasir::Register::isSet(KSR::PCLKSR::osc8mrdy));
        apply(GenericClockGenerator<0, GeneratorSource::osc8m, 1>::enable());
        Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
          Kvasir::Register::isClear(GC::STATUS::syncbusy));
        apply(set(KSR::OSC8M::ondemand));
    }
}}   // namespace Kvasir::GCLK
