#pragma once
// The SAM D21's start traits for atsam_common/StartUp.hpp's FirstInitStep: whether the chip looks as a reset leaves it
// (restartIfNotFromReset() restarts an image at address 0 that was jumped into), and GCLK0 parked on the clock a reset
// gives it before the clock tree is rewritten.

#include "GCLK.hpp"

namespace Kvasir { namespace Startup { namespace Sam {
    struct StartTraits {
        // Whether the chip looks as a reset leaves it: no APBC bus clock but the ADC's (PM.APBCMASK resets to
        // 0x00010000, 16.8.10) and GCLK0 on OSC8M (8.3.1). Any image that used a SERCOM, TC, TCC, DAC or AC, or ran
        // GCLK0 on another source, leaves a difference behind - and a jump does not reset either (SysResetReq and WDT
        // reset all but the RTC, 32 kHz sources, an ALWAYSON WDT and WRTLOCKed GCLKs, Table 16-2).
        static bool looksLikeAfterReset() {
            using GCLK::GeneratorSource;
            auto const apbc = Kvasir::Register::readWhole(
              Kvasir::Peripheral::PM::Registers<>::APBCMASK::FULLREGISTER);
            return apbc == 0x0001'0000U && GCLK::gclk0Source() == GeneratorSource::osc8m;
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
        static void parkMainClock() {
            using GCLK::GeneratorSource;
            using GC          = Kvasir::Peripheral::GCLK::Registers<>;
            using KSR         = Kvasir::Peripheral::SYSCTRL::Registers<>;
            auto const source = GCLK::gclk0Source();
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
            apply(GCLK::GenericClockGenerator<0, GeneratorSource::osc8m, 1>::enable());
            Kvasir::Register::waitUntil<Kvasir::Chip::Sam::SyncBound>(
              Kvasir::Register::isClear(GC::STATUS::syncbusy));
            apply(set(KSR::OSC8M::ondemand));
        }
    };
}}}   // namespace Kvasir::Startup::Sam
