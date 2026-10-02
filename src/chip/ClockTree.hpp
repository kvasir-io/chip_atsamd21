#pragma once
// The SAM D21's core clock from a target, like DefaultClockSettings on the RP: the flash wait
// states from the datasheet table, and
// the whole tree crystal -> FDPLL96M -> generator 0 from a target frequency.
//
//     struct ClockSettings {
//         using Tree = Kvasir::Clocks::Sam::Tree<{.cpu    = 48'000'000,
//                                                 .source = {.hz = 8'000'000}}>;
//         using Provides = Tree::Provides;   // ProcessorClock<48 MHz>, Clk<ClkMain, 48 MHz>
//         static void coreClockInit() { Tree::coreClockInit(); }
//     };
//     static_assert(ClockSettings::Tree::Dpll.div == 3 && ClockSettings::Tree::Dpll.ldr == 47);
//
// The wait states alone, for a firmware with its own tree (every DFLL firmware):
//
//     apply(Kvasir::Nvm::setWaitStates<Kvasir::Nvm::waitStates<48'000'000, Supply::from2V7>()>());

#include "ClockLimits.hpp"
#include "DFLL.hpp"
#include "Dpll.hpp"
#include "GCLK.hpp"
#include "atsam_common/Clocks.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/NVMCTRL.hpp"

#include <cstdint>

namespace Kvasir { namespace Nvm {
    using ClockLimits::Supply;
    using Rating = ClockLimits::D21::Rating;
    using ClockLimits::D21::waitStates;

    // NVMCTRL.CTRLB.RWS = Ws, the rest of CTRLB at the SVD's reset value (MANW = 1, 22.8.2, md
    // line 16163). Written through the generated field with a cast: the SVD names only 0..2, the
    // field is 4 bits (22.8.2, md line 16110) and a 1.62 V board at 48 MHz needs 3 (Table 37-42).
    // RWS must match the new frequency before the AHB clock goes up (22.5.2, line 15653).
    template<unsigned Ws>
    [[nodiscard]] constexpr auto setWaitStates() {
        static_assert(Ws <= ClockLimits::D21::RwsMax, "NVMCTRL.CTRLB.RWS is 4 bits");
        using KNR = Kvasir::Peripheral::NVMCTRL::Registers<>;
        using RV  = typename KNR::CTRLB::RWSVal;
        return KNR::CTRLB::overrideDefaults(
          write(KNR::CTRLB::rws, Kvasir::Register::value<RV, static_cast<RV>(Ws)>()));
    }

    // A count written by hand, checked: "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 (SAM
    // D21 Table 37-42), RWS is 1"
    template<std::uint64_t Hz,
             Supply        S,
             unsigned      Rws,
             Rating        R = Rating::c85>
    consteval void assertWaitStates() {
        static_assert(R == Rating::c85 || R == Rating::c125 || R == Rating::aecQ100VariantA
                        || R == Rating::aecQ100VariantBD,
                      "SAM D21 at 105 C: no wait-state table (DS40001882L chapter 38)");
        if constexpr(R == Rating::c85) {
            assertWaitStatesFrom<Hz, S, Rws, ClockLimits::D21::WaitStates85C>();
        } else if constexpr(R == Rating::c125) {
            assertWaitStatesFrom<Hz, S, Rws, ClockLimits::D21::WaitStates125C>();
        } else if constexpr(R == Rating::aecQ100VariantA) {
            assertWaitStatesFrom<Hz, S, Rws, ClockLimits::D21::WaitStatesAecQ100A>();
        } else if constexpr(R == Rating::aecQ100VariantBD) {
            assertWaitStatesFrom<Hz, S, Rws, ClockLimits::D21::WaitStatesAecQ100BD>();
        }
    }
}}   // namespace Kvasir::Nvm

namespace Kvasir { namespace Clocks { namespace Sam {
    struct Xosc {
        std::uint64_t hz{};
        bool          crystal = true;
    };

    struct TreeConfig {
        std::uint64_t            cpu{};
        Xosc                     source{};
        ClockLimits::Supply      supply         = ClockLimits::Supply::from2V7;   // board fact
        ClockLimits::D21::Rating rating         = ClockLimits::D21::Rating::c85;
        std::uint64_t            tolerancePpm   = 0;
        bool                     dpllRunStandby = false;
        bool                     dpllLockBypass = true;   // errata 1.3.5 (Dpll.hpp)
        XOSC::Options            xosc           = {};
    };

    // XOSC -> FDPLL96M (solver: lowest DCO, integer mode) -> RWS for the new clock -> generator 0.
    // Order as on the RP (clock_config.hpp): oscillator up, PLL running (CLKRDY), flash timing for
    // the faster clock, then the switch (22.5.2).
    template<TreeConfig C>
    struct Tree {
        static_assert(C.cpu <= ClockLimits::D21::CpuMax,
                      "SAM D21: f_CPU max 48 MHz (Table 37-7)");

        static constexpr DPLL::Setting Dpll = DPLL::fromXosc<C.source.hz, C.cpu, C.tolerancePpm>();
        static constexpr unsigned      WaitStates = Nvm::waitStates<C.cpu, C.supply, C.rating>();

        // the DCO above 64 MHz does not work below 0 C on revisions A-D (errata 1.3.2): the solver
        // takes the lowest DCO, so this only fires for a target that needs more
        static_assert(!Prescaler::above(Dpll.dco,
                                        ClockLimits::D21::DcoMaxBelow0C),
                      "FDPLL96M DCO above 64 MHz: not functional below 0 C (errata DS80000760M "
                      "1.3.2, revisions A-D)");

        static constexpr XOSC::Options XoscOptions = [] {
            auto o    = C.xosc;
            o.crystal = C.source.crystal;
            return o;
        }();

        using Provides = brigand::list<Clk<ClkMain, C.cpu>, Startup::ProcessorClock<C.cpu>>;

        static void coreClockInit() {
            apply(XOSC::configure<C.source.hz, XoscOptions>());
            while(!XOSC::ready()) {}

            apply(DPLL::configure<Dpll,
                                  DPLL::Reference::xosc,
                                  DPLL::Options{.lockBypass = C.dpllLockBypass,
                                                .runStandby = C.dpllRunStandby}>());
            while(!DPLL::ready()) {}

            apply(
              Nvm::setWaitStates<WaitStates>(),
              GCLK::GenericClockGenerator<0, GCLK::GeneratorSource::fdpll, Dpll.gclkDiv>::enable());
        }
    };
}}}   // namespace Kvasir::Clocks::Sam
