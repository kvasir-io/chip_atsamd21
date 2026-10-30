#pragma once
// The SAM D21's core clock from a target, like DefaultClockSettings on the RP: the flash wait
// states from the datasheet table, and the whole tree from a target frequency and a source kind
// (atsam_common/ClockTree.hpp's skeleton over this chip's builders):
//
//     struct ClockSettings {
//         using Tree = Kvasir::Clocks::Sam::Tree<{.cpu    = 48'000'000,
//                                                 .source = {.hz = 8'000'000}}>;   // XOSC -> FDPLL96M
//         using Provides = Tree::Provides;   // ProcessorClock<48 MHz>, Hal::Clk<ClkMain, 48 MHz>
//         static void coreClockInit() { Tree::coreClockInit(); }
//     };
//     static_assert(ClockSettings::Tree::Dpll.div == 3 && ClockSettings::Tree::Dpll.ldr == 47);
//
// The DFLL48M kinds, for a board without a crystal:
//
//     Tree<{.cpu = 48'000'000, .kind = Source::dfllOpenLoop}>      // factory coarse value, 45..49 MHz
//     Tree<{.cpu = 48'000'000, .kind = Source::dfllUsbRecovery}>   // USB SOF, also GCLK_USB (G18A)
//     Tree<{.cpu = 48'000'000, .kind = Source::dfllClosedLoop, .reference = {.accuracyPpm = 19'500}}>
//
// The crystal with a way out (a board that must start with a dead crystal too):
//
//     Tree<{.cpu = 48'000'000, .kind = Source::xoscFdpllElseDfll, .source = {.hz = 8'000'000}}>   // Tree::onCrystal()
//
// The wait states alone, for a firmware with its own tree:
//
//     apply(Kvasir::Nvm::setWaitStates<Kvasir::Nvm::waitStates<48'000'000, Supply::from2V7>()>());

#include "ClockLimits.hpp"
#include "DFLL.hpp"
#include "Dpll.hpp"
#include "GCLK.hpp"
#include "PM.hpp"
#include "WaitBounds.hpp"
#include "atsam_common/ClockTree.hpp"
#include "atsam_common/Clocks.hpp"
#include "core_peripherals/SYSTICK.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "peripherals/SYSCTRL.hpp"
#include "peripherals/TC_COUNT16.hpp"

#include <cstdint>
#include <type_traits>

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
        static_assert(Ws <= ClockLimits::D21::rwsMax, "NVMCTRL.CTRLB.RWS is 4 bits");
        using KNR = Kvasir::Peripheral::NVMCTRL::Registers<>;
        using RV  = typename KNR::CTRLB::RWSVal;
        return KNR::CTRLB::overrideDefaults(
          write(KNR::CTRLB::rws, Kvasir::Register::value<RV, static_cast<RV>(Ws)>()));
    }
}}   // namespace Kvasir::Nvm

namespace Kvasir { namespace Clocks { namespace Sam {
    // The closed loop's reference: a generator (not 0, the one being switched) from `source` at `sourceHz`,
    // divided by `div`, on GCLK_DFLL48M_REF (CLKCTRL ID 0, GCLK.hpp). The default is OSC8M, whose PRESC resets
    // to /8 (DS40001882L 17.8.9: 1 MHz), / 32 = 31.25 kHz, inside f_REF 0.732..33 kHz (Table 37-54) - the
    // reference a board without a crystal has. `accuracyPpm` has no default: the board states its reference's
    // accuracy, which must be within 2 % (note 1 under Table 37-54; OSC8M only at 3.3 V, Table 37-57; OSCULP32K
    // never, Table 37-56). A board with a 32 kHz crystal passes source = xosc32k, sourceHz = 32'768, div = 1.
    struct DfllReference {
        unsigned              generator = 1;
        GCLK::GeneratorSource source    = GCLK::GeneratorSource::osc8m;
        std::uint64_t         sourceHz  = 1'000'000;
        unsigned long long    div       = 32;
        std::uint64_t         accuracyPpm{};
    };

    struct TreeConfig {
        std::uint64_t            cpu{};
        Source                   kind = Source::xoscFdpll;
        Xosc                     source{};                                        // xoscFdpll
        DfllReference            reference{};                                     // dfllClosedLoop
        ClockLimits::Supply      supply         = ClockLimits::Supply::from2V7;   // board fact
        ClockLimits::D21::Rating rating         = ClockLimits::D21::Rating::c85;
        std::uint64_t            tolerancePpm   = 0;
        bool                     dpllRunStandby = false;
        bool                     dpllLockBypass = true;   // errata 1.3.5 (Dpll.hpp)
        XOSC::Options            xosc           = {};
        bool usbClock = true;   // dfllUsbRecovery: route GCLK_USB (a main with the USB)
    };

    template<TreeConfig C, Source = C.kind>
    struct D21Kind;

    // XOSC -> FDPLL96M (solver: lowest DCO, integer mode) -> RWS for the new clock -> generator 0.
    // Order as on the RP (clock_config.hpp): oscillator up, PLL running (CLKRDY), flash timing for
    // the faster clock, then the switch (22.5.2).
    template<TreeConfig C>
    struct D21Kind<C, Source::xoscFdpll> {
        static constexpr DPLL::Setting dpll = DPLL::fromXosc<C.source.hz, C.cpu, C.tolerancePpm>();

        // the DCO above 64 MHz does not work below 0 C on revisions A-D (errata 1.3.2): the solver
        // takes the lowest DCO, so this only fires for a target that needs more
        static_assert(!Prescaler::above(dpll.dco,
                                        ClockLimits::D21::dcoMaxBelow0C),
                      "FDPLL96M DCO above 64 MHz: not functional below 0 C (errata DS80000760M "
                      "1.3.2, revisions A-D)");

        static constexpr XOSC::Options xoscOptions = [] {
            auto o    = C.xosc;
            o.crystal = C.source.crystal;
            return o;
        }();

        using ExtraProvides = brigand::list<>;

        template<unsigned WaitStates>
        static void start() {
            apply(XOSC::configure<C.source.hz, xoscOptions>());
            Kvasir::Register::waitUntil<Kvasir::Chip::Sam::XoscReadyBound<xoscOptions.startup>>(
              [] { return XOSC::ready(); });

            apply(DPLL::configure<dpll,
                                  DPLL::Reference::xosc,
                                  DPLL::Options{.lockBypass = C.dpllLockBypass,
                                                .runStandby = C.dpllRunStandby}>());
            Kvasir::Register::waitUntil<Kvasir::Chip::DpllLockBound>([] { return DPLL::ready(); });

            // RWS first, then the switch: two apply() calls, so the order is the source's (one apply orders its writes
            // by address, Register/Apply.hpp; NVMCTRL 0x41004004 above GCLK 0x40000C04 made it the same)
            apply(Nvm::setWaitStates<WaitStates>());
            apply(
              GCLK::GenericClockGenerator<0, GCLK::GeneratorSource::fdpll, dpll.gclkDiv>::enable());
        }
    };

    namespace Detail {
        // Is the FDPLL96M's output the frequency it was set for? Its LOCK and CLKRDY are no proof: on a board without
        // a crystal XOSCRDY (the start-up counter, 17.6.2) and CLKRDY both came, and the DCO ran free at ~31.9 MHz
        // (measured on a G18A rev D). So count it: generator 1 takes the FDPLL / 16 to TC3 (GCLK channel
        // TCC2_TC3, 15.8.3), and SysTick on the processor clock times a window while the CPU still runs from OSC8M
        // (GCLK0 undivided, from reset or parkMainClock: 8 MHz >> SYSCTRL.OSC8M.PRESC, 17.8.9 md l.8884). A window of
        // f_CPU / 250 SysTick ticks is 4 ms, so the count expected is (FDPLL / 16) / 250 whatever PRESC is. OSC8M is
        // within 7.8..8.16 MHz over the whole range (Table 37-57): +-10 % takes that and the read-request latency, and
        // still tells 31.9 MHz from 48. Afterwards TC3, its channel, generator 1, TC3's APB gate and SysTick are what a
        // reset leaves: TC3 by CTRLA.SWRST (30.8.1), CLKCTRL ID 0x1B with CLKEN 0 and GEN 0 (Table 15-3 "Others"),
        // GENCTRL / GENDIV of generator 1 = 0x00000001 (15.8.4 / 15.8.5 power-reset tables), APBCMASK.TC3 0 (16.8.10,
        // reset 0x00010000), SYST_CSR.ENABLE 0. Registers only: it runs in coreClockInit.
        template<std::uint64_t FdpllHz>
        struct FdpllCheck {
            static constexpr std::uint32_t div = 16;
            static constexpr std::uint32_t expected
              = static_cast<std::uint32_t>(FdpllHz / div / 250);
            static_assert(expected > 100 && expected * 11 / 10 < 0x1'0000,
                          "FdpllCheck: the count of the 4 ms window fits TC3's 16 bits");
            using Bound = Kvasir::Register::Bound::Polls<100'000>;

            [[nodiscard]] static bool matches() {
                namespace R         = Kvasir::Register;
                using Tc            = Kvasir::Peripheral::TC_COUNT16::Registers<3>;
                using KSR           = Kvasir::Peripheral::SYSCTRL::Registers<>;
                using GC            = Kvasir::Peripheral::GCLK::Registers<>;
                using Syst          = Kvasir::Peripheral::SYSTICK::Registers<>;
                auto const tcSynced = [] {
                    return R::waitUntil<Bound, R::OnTimeout::Report>(
                      R::isClear(Tc::STATUS::syncbusy));
                };
                auto const gclkSynced = [] {
                    return R::waitUntil<Bound, R::OnTimeout::Report>(
                      R::isClear(GC::STATUS::syncbusy));
                };
                // COUNT through a read request (READREQ.RREQ, ADDR = COUNT's offset 0x10, 30.8.2)
                auto const count = [&] {
                    apply(Tc::READREQ::overrideDefaults(
                      write(Tc::READREQ::addr, R::value<std::uint16_t, 0x10>()),
                      set(Tc::READREQ::rreq)));
                    static_cast<void>(tcSynced());
                    return static_cast<std::uint32_t>(get<0>(apply(read(Tc::COUNT::count))));
                };

                apply(typename PM::Enable<Tc::baseAddr>::action{});
                apply(GCLK::GenericClockGenerator<1, GCLK::GeneratorSource::fdpll, div>::enable());
                static_cast<void>(gclkSynced());
                apply(GCLK::PeripheralChannelController<1, GCLK::Peripheral::tcc2_tc3>::enable());
                apply(set(Tc::CTRLA::swrst));
                bool ok = static_cast<bool>(tcSynced());
                apply(write(Tc::CTRLA::MODEValC::count16), set(Tc::CTRLA::enable));
                ok = ok && static_cast<bool>(tcSynced());

                std::uint32_t const cpuHz
                  = 8'000'000U >> static_cast<unsigned>(get<0>(apply(read(KSR::OSC8M::presc))));
                std::uint32_t const window = cpuHz / 250;
                apply(write(Syst::RVR::reload, 0x00FF'FFFFU));
                apply(write(Syst::CVR::current, 0U));
                apply(write(Syst::CSR::CLKSOURCEValC::processor), set(Syst::CSR::enable));
                std::uint32_t const t0    = get<0>(apply(read(Syst::CVR::current)));
                std::uint32_t const first = count();
                bool const timed = static_cast<bool>(R::waitUntil<Bound, R::OnTimeout::Report>([&] {
                    return ((t0 - get<0>(apply(read(Syst::CVR::current)))) & 0x00FF'FFFFU)
                        >= window;
                }));
                std::uint32_t const counted = (count() - first) & 0xFFFFU;

                // back to what a reset leaves
                apply(clear(Syst::CSR::enable));
                apply(set(Tc::CTRLA::swrst));
                static_cast<void>(tcSynced());
                apply(GC::CLKCTRL::overrideDefaults(
                  write(GC::CLKCTRL::id,
                        R::value<GCLK::Peripheral, GCLK::Peripheral::tcc2_tc3>())));
                apply(GC::GENCTRL::overrideDefaults(write(GC::GENCTRL::id, R::value<1>())));
                static_cast<void>(gclkSynced());
                apply(GC::GENDIV::overrideDefaults(write(GC::GENDIV::id, R::value<1>())));
                apply(typename PM::Disable<Tc::baseAddr>::action{});

                return ok && timed && counted * 10 >= expected * 9 && counted * 10 <= expected * 11;
            }
        };

        // Generator 0 on the DFLL48M for f_CPU: 48 MHz / Div. Div = 1 for 48 MHz (DIVSEL 0 and GENDIV.DIV 0 or 1
        // are both "not divided", 15.8.4). The wait states are those of the 48 MHz row: the open loop runs up to
        // 49 MHz (Table 37-52, ClockLimits::D21::dfllOpenLoopMaxHz), above f_CPU max (Table 37-7) and above the last
        // row of Table 37-42, which has no count for it - what every DFLL firmware takes; the closed loop and USB
        // recovery stay at 48 MHz +-0.25 % (Table 37-54, 32.5.3). Divided: D21Builders::flashHz.
        template<std::uint64_t Cpu>
        struct DfllGclk0 {
            static_assert(Cpu != 0 && 48'000'000 % Cpu == 0,
                          "a DFLL48M tree runs the CPU at 48 MHz or an integer fraction of it "
                          "(generator 0 divides)");
            static constexpr unsigned long long div = 48'000'000 / Cpu;

            // RWS first, then the switch (22.5.2): two apply() calls, the order the source's
            template<unsigned WaitStates>
            static void switchOver() {
                apply(Nvm::setWaitStates<WaitStates>());
                apply(
                  GCLK::GenericClockGenerator<0, GCLK::GeneratorSource::dfll48m, div>::enable());
            }
        };
    }   // namespace Detail

    // DFLL48M in open loop on the factory's coarse value (DFLL.hpp: 45 / 47 / 49 MHz, Table 37-52)
    template<TreeConfig C>
    struct D21Kind<C, Source::dfllOpenLoop> {
        using ExtraProvides = brigand::list<>;

        template<unsigned WaitStates>
        static void start() {
            DFLL::enableOpenLoop();
            Detail::DfllGclk0<C.cpu>::template switchOver<WaitStates>();
        }
    };

    // DFLL48M locked to the USB start-of-frame (DFLL.hpp, 17.6.7.2.2): the CPU and GCLK_USB at 48 MHz +-0.25 %
    // (32.5.3) from generator 0. Until a host sends SOFs it runs as in open loop. GCLK_USB is routed here (CLKCTRL
    // ID 6, 15.8.3) and the tree provides ClkUsb - unless `.usbClock = false` (a main without the USB device: the
    // channel stays off); the USB exists on the G18A only.
    template<TreeConfig C>
    struct D21Kind<C, Source::dfllUsbRecovery> {
        static_assert(C.cpu == 48'000'000,
                      "USB recovery: generator 0 also feeds GCLK_USB, which needs 48 MHz");
#ifdef KVASIR_CHIP_ATSAMD21G18A
        using ExtraProvides = std::
          conditional_t<C.usbClock, brigand::list<Hal::Clk<ClkUsb, 48'000'000>>, brigand::list<>>;

        template<unsigned WaitStates>
        static void start() {
            DFLL::enableUsbRecovery();
            Detail::DfllGclk0<C.cpu>::template switchOver<WaitStates>();
            if constexpr(C.usbClock) {
                apply(GCLK::PeripheralChannelController<0, GCLK::Peripheral::usb>::enable());
            }
        }
#endif
    };

    // DFLL48M locked to a reference generator (DFLL::enableClosedLoop, 17.6.7.1.2): the reference generator, its
    // channel GCLK_DFLL48M_REF, the loop (which checks f_REF, MUL and the 2 % itself), then generator 0. The lock
    // wait is enableClosedLoop's DfllLockBound. No DFLLOOB interrupt (errata 1.2.2, DFLL.hpp).
    template<TreeConfig C>
    struct D21Kind<C, Source::dfllClosedLoop> {
        static constexpr DfllReference ref = C.reference;
        static_assert(ref.generator != 0 && ref.generator < 9,
                      "the DFLL48M reference needs a generator of its own: 1..8, not 0 (the one "
                      "being switched)");
        static_assert(ref.accuracyPpm != 0,
                      "the DFLL48M closed loop needs the reference's accuracy: .reference = "
                      "{.accuracyPpm = ...} "
                      "(note 1 under SAM D21 Table 37-54: within 2 %)");
        static_assert(ref.div != 0 && ref.sourceHz % ref.div == 0,
                      "the reference divides its source exactly");
        static constexpr std::uint64_t refHz = ref.sourceHz / ref.div;

        using ExtraProvides = brigand::list<>;

        template<unsigned WaitStates>
        static void start() {
            apply(GCLK::GenericClockGenerator<ref.generator, ref.source, ref.div>::enable());
            apply(
              GCLK::PeripheralChannelController<ref.generator, GCLK::Peripheral::dfll48>::enable());
            DFLL::enableClosedLoop<refHz, ref.accuracyPpm, 48'000'000>();
            Detail::DfllGclk0<C.cpu>::template switchOver<WaitStates>();
        }
    };

    // xoscFdpll with a way out: the crystal or the FDPLL that does not come in a bounded wait (1 000 000 polls each,
    // OnTimeout::Report whatever the image's WaitPolicy - at the 1 MHz the CPU runs on after a reset, OSC8M / 8, some
    // seconds; the crystal's own STARTUP 15 is ~1 s, 17.8.5 l.8456) turns XOSC and FDPLL off again and generator 0
    // goes to the DFLL48M in open loop (DFLL.hpp: 45..49 MHz, Table 37-52). onCrystal() tells the two apart from the
    // registers (DPLLCTRLA.ENABLE), as coreClockInit runs before .data and .bss: no variable. The wait states are those
    // of the worse branch (D21Builders::flashHz). For a board that must still be heard from with a dead crystal - the
    // DFLL's tolerance is the UARTs' and every crystal-timed peripheral's problem then.
    template<TreeConfig C>
    struct D21Kind<C, Source::xoscFdpllElseDfll> : D21Kind<C, Source::xoscFdpll> {
        using Base = D21Kind<C, Source::xoscFdpll>;
        using Base::dpll;
        using Base::xoscOptions;
        using ExtraProvides = brigand::list<>;
        using CrystalBound  = Kvasir::Register::Bound::Polls<1'000'000>;

        [[nodiscard]] static bool onCrystal() {
            return apply(read(Kvasir::Peripheral::SYSCTRL::Registers<>::DPLLCTRLA::enable)) != 0U;
        }

        template<unsigned WaitStates>
        static void start() {
            namespace R = Kvasir::Register;
            using KSR   = Kvasir::Peripheral::SYSCTRL::Registers<>;
            apply(XOSC::configure<C.source.hz, xoscOptions>());
            bool crystal = static_cast<bool>(
              R::waitUntil<CrystalBound, R::OnTimeout::Report>([] { return XOSC::ready(); }));
            if(crystal) {
                apply(DPLL::configure<dpll,
                                      DPLL::Reference::xosc,
                                      DPLL::Options{.lockBypass = C.dpllLockBypass,
                                                    .runStandby = C.dpllRunStandby}>());
                crystal = static_cast<bool>(
                  R::waitUntil<CrystalBound, R::OnTimeout::Report>([] { return DPLL::ready(); }));
            }
            // ready is not running: the FDPLL counted against OSC8M (Detail::FdpllCheck)
            crystal = crystal && Detail::FdpllCheck<C.cpu * dpll.gclkDiv>::matches();
            // RWS first (22.5.2), its own apply(): before either switch
            apply(Nvm::setWaitStates<WaitStates>());
            if(crystal) {
                apply(GCLK::GenericClockGenerator<0, GCLK::GeneratorSource::fdpll, dpll.gclkDiv>::
                        enable());
            } else {
                apply(KSR::DPLLCTRLA::overrideDefaults(clear(KSR::DPLLCTRLA::enable)),
                      KSR::XOSC::overrideDefaults(clear(KSR::XOSC::enable)));
                DFLL::enableOpenLoop();
                apply(GCLK::GenericClockGenerator<0,
                                                  GCLK::GeneratorSource::dfll48m,
                                                  Detail::DfllGclk0<C.cpu>::div>::enable());
            }
        }
    };

    struct D21Builders {
        using Config = TreeConfig;

        static consteval bool has(Source s) {
#ifdef KVASIR_CHIP_ATSAMD21G18A
            return s == Source::xoscFdpll || s == Source::dfllOpenLoop
                || s == Source::dfllUsbRecovery || s == Source::dfllClosedLoop
                || s == Source::xoscFdpllElseDfll;
#else
            return s == Source::xoscFdpll || s == Source::dfllOpenLoop
                || s == Source::dfllClosedLoop || s == Source::xoscFdpllElseDfll;
#endif
        }

        static consteval std::uint64_t cpuMax(TreeConfig const&) {
            return ClockLimits::D21::cpuMax;
        }

        // The clock the flash sees at worst. A DFLL48M divided by generator 0 runs up to 49 MHz / Div (open loop,
        // Table 37-52 max; the closed loop's 2 % reference keeps it below that, note 1 under Table 37-54), so
        // 24 MHz from 45..49 MHz is up to 24.5 MHz, past the 0-wait-state row of Table 37-42 (24 MHz at 2.7 V).
        // Undivided it takes the 48 MHz row (Detail::DfllGclk0). An FDPLL output is locked to its crystal. A cpu
        // that does not divide 48 MHz is DfllGclk0's refusal: f_CPU here, so that its message is the only one.
        template<TreeConfig C>
        static consteval std::uint64_t flashHz() {
            if(C.kind == Source::xoscFdpll || C.cpu == 0 || C.cpu >= 48'000'000
               || 48'000'000 % C.cpu != 0)
            {
                return C.cpu;
            }
            std::uint64_t const div = 48'000'000 / C.cpu;
            return (ClockLimits::D21::dfllOpenLoopMaxHz + div - 1) / div;
        }

        template<TreeConfig C>
        static consteval unsigned waitStates() {
            return Nvm::waitStates<flashHz<C>(), C.supply, C.rating>();
        }

        template<TreeConfig C>
        using Kind = D21Kind<C>;
    };

    // f_CPU max 48 MHz (Table 37-7)
    template<TreeConfig C>
    using Tree = TreeOf<D21Builders, C>;
}}}   // namespace Kvasir::Clocks::Sam
