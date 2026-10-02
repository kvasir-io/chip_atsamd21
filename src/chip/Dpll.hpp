#pragma once
// The SAM D21's crystal oscillator and FDPLL96M as building blocks: the numbers come from the
// solver (atsam_common/ClockSolver.hpp on ClockLimits.hpp), the register writes are these. A
// firmware with its own sequence (suntrace_pro's crystal fallback) uses them one by one;
// ClockTree.hpp's Tree puts them together.
//
//     constexpr auto dpll = Kvasir::DPLL::fromXosc<8'000'000, 48'000'000>();   // DIV 3, LDR 47
//     apply(Kvasir::XOSC::configure<8'000'000>());
//     while(!Kvasir::XOSC::ready()) {}
//     apply(Kvasir::DPLL::configure<dpll>());       // LBYPASS on: errata 1.3.5
//     while(!Kvasir::DPLL::ready()) {}

#include "ClockLimits.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/SYSCTRL.hpp"

#include <cstdint>

namespace Kvasir { namespace XOSC {
    struct Options {
        bool crystal    = true;   // XTALEN: a crystal between XIN and XOUT, else a clock on XIN
        bool ampgc      = true;   // automatic amplitude gain control
        bool runStandby = true;
        std::uint16_t startup
          = 15;   // XOSC.STARTUP 0xF: 32768 OSCULP32K cycles, ~1 s (17.8.5, line 8456)
    };

    // XOSC.GAIN: the "recommended max frequency" per setting, 2 / 4 / 8 / 16 / 30 MHz (SAM D21
    // DS40001882L 17.8.5 "Bits 10:8 - GAIN", md line 8506); it "must be properly configured even
    // when the Automatic Amplitude Gain Control is active".
    consteval std::uint16_t gainFor(std::uint64_t hz) {
        if(hz <= 2'000'000) { return 0; }
        if(hz <= 4'000'000) { return 1; }
        if(hz <= 8'000'000) { return 2; }
        if(hz <= 16'000'000) { return 3; }
        return 4;
    }

    // XOSC enabled, ONDEMAND off (it runs whether or not a generator asks for it)
    template<std::uint64_t Hz,
             Options       O = {}>
    [[nodiscard]] constexpr auto configure() {
        // crystal 0.4..32 MHz (Table 37-48, line 42022); XIN clock up to 32 MHz (Table 37-47,
        // 41988)
        static_assert(Hz <= 32'000'000 && (!O.crystal || Hz >= 400'000),
                      "SAM D21 XOSC: a crystal of 0.4..32 MHz (Table 37-48) or an XIN clock up to "
                      "32 MHz (Table 37-47)");
        using KSR = Kvasir::Peripheral::SYSCTRL::Registers<>;
        using Kvasir::Register::value;
        using G = typename KSR::XOSC::GAINVal;
        return KSR::XOSC::overrideDefaults(
          write(KSR::XOSC::startup, value<std::uint16_t, O.startup>()),
          write(KSR::XOSC::ampgc, value<std::uint16_t, O.ampgc ? 1U : 0U>()),
          write(KSR::XOSC::gain, value<G, static_cast<G>(gainFor(Hz))>()),
          clear(KSR::XOSC::ondemand),
          write(KSR::XOSC::runstdby, value<std::uint16_t, O.runStandby ? 1U : 0U>()),
          write(KSR::XOSC::xtalen, value<std::uint16_t, O.crystal ? 1U : 0U>()),
          set(KSR::XOSC::enable));
    }

    [[nodiscard]] inline bool ready() {
        using KSR = Kvasir::Peripheral::SYSCTRL::Registers<>;
        return apply(read(KSR::PCLKSR::xoscrdy));
    }
}}   // namespace Kvasir::XOSC

namespace Kvasir { namespace DPLL {
    // DPLLCTRLB.REFCLK (17.8.19, line 9498): REF0 = XOSC32K, REF1 = XOSC (through DIV), GCLK_DPLL
    enum class Reference : unsigned { xosc32k = 0, xosc = 1, gclk = 2 };

    struct Options {
        // LBYPASS: "below 25 C a spurious DPLL unlock may be detected ... the DPLL output clock is
        // halted and then restarts"; the workaround is the lock bypass, set before ENABLE (errata
        // DS80000760M 1.3.5, all revisions A-J, line 490, PDF page 10). Costs nothing: on by
        // default.
        bool lockBypass = true;
        bool runStandby = false;
    };

    // The solver for this chip, asserted: a crystal on XOSC through DIV
    template<std::uint64_t XoscHz,
             std::uint64_t Hz,
             std::uint64_t TolerancePpm = 0,
             bool          Fractional   = false>
    consteval Setting fromXosc() {
        return solveXosc<ClockLimits::D21::Fdpll96m,
                         ClockLimits::D21::DpllWhere,
                         XoscHz,
                         Hz,
                         TolerancePpm,
                         Fractional>();
    }

    // ... and a reference taken directly (XOSC32K, a generator)
    template<std::uint64_t RefHz,
             std::uint64_t Hz,
             std::uint64_t TolerancePpm = 0,
             bool          Fractional   = false>
    consteval Setting fromReference() {
        return solveReference<ClockLimits::D21::Fdpll96m,
                              ClockLimits::D21::DpllWhere,
                              RefHz,
                              Hz,
                              TolerancePpm,
                              Fractional>();
    }

    // DPLLRATIO, DPLLCTRLB, then DPLLCTRLA with ENABLE and ONDEMAND off; the caller waits for
    // ready() and only then switches a generator over (17.6.8.4, line 7433).
    template<Setting   S,
             Reference R = Reference::xosc,
             Options   O = {}>
    [[nodiscard]] constexpr auto configure() {
        static_assert(S.found, "DPLL::configure: a setting the solver did not find");
        static_assert(S.presc == 0, "the SAM D21 FDPLL96M has no output prescaler");
        static_assert(R == Reference::xosc || S.div == 0, "DIV only divides the XOSC reference");
        using KSR = Kvasir::Peripheral::SYSCTRL::Registers<>;
        using Kvasir::Register::value;
        using RV         = typename KSR::DPLLCTRLB::REFCLKVal;
        auto const ratio = list(write(KSR::DPLLRATIO::ldr, value<S.ldr>()),
                                write(KSR::DPLLRATIO::ldrfrac, value<S.ldrFrac>()));
        auto const ctrlb = [] {
            if constexpr(O.lockBypass) {
                return KSR::DPLLCTRLB::overrideDefaults(
                  write(KSR::DPLLCTRLB::div, value<S.div>()),
                  set(KSR::DPLLCTRLB::lbypass),
                  write(KSR::DPLLCTRLB::refclk, value<RV, static_cast<RV>(R)>()));
            } else {
                return KSR::DPLLCTRLB::overrideDefaults(
                  write(KSR::DPLLCTRLB::div, value<S.div>()),
                  write(KSR::DPLLCTRLB::refclk, value<RV, static_cast<RV>(R)>()));
            }
        }();
        auto const ctrla = [] {
            if constexpr(O.runStandby) {
                return KSR::DPLLCTRLA::overrideDefaults(clear(KSR::DPLLCTRLA::ondemand),
                                                        set(KSR::DPLLCTRLA::enable),
                                                        set(KSR::DPLLCTRLA::runstdby));
            } else {
                return KSR::DPLLCTRLA::overrideDefaults(clear(KSR::DPLLCTRLA::ondemand),
                                                        set(KSR::DPLLCTRLA::enable));
            }
        }();
        return list(ratio, ctrlb, Kvasir::Register::sequencePoint, ctrla);
    }

    // CLK_FDPLL96M runs (DPLLSTATUS.CLKRDY; with LBYPASS the lock bit no longer gates it)
    [[nodiscard]] inline bool ready() {
        using KSR = Kvasir::Peripheral::SYSCTRL::Registers<>;
        return apply(read(KSR::DPLLSTATUS::clkrdy));
    }

    // configure() and wait for ready()
    template<Setting   S,
             Reference R = Reference::xosc,
             Options   O = {}>
    inline void enable() {
        apply(configure<S, R, O>());
        while(!ready()) {}
    }
}}   // namespace Kvasir::DPLL
