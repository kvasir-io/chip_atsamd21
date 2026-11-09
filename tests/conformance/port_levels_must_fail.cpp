// A SAM pin's level (or direction) written twice in one apply() as must-fail cases (kvasir_conformance_must_fail,
// CMakeLists.txt): one step runs by register address, highest first, so `makeOutput(pin)` (OUTCLR) with `set(pin)`
// (OUTSET, +4 above it) ended LOW. chip_atsam_common RegisterSeam.hpp makes PORT's DIR and OUT shared states
// (Register::SharedState) and apply() refuses two of their registers on one pin in one step, naming the pin.
// PORT_FAIL=0 must compile: a Startup whose entries give the level in the configuring action (makeOutputInitHigh), a
// list with makeOutput and clear of one pin (one register), two pins' levels in one list, and makeOutput then set in two
// apply() calls or two steps. Each of 1..7 must fail with the pin's name and what it writes (the case list in
// CMakeLists.txt): direct apply() lists, an entry's initStepPinConfig, and two entries of one Startup phase.

#include "kvasir_config.hpp"
//
#include <chip/ClockTree.hpp>
#include <chip/chip.hpp>
#include <kvasir/Runtime/BootReport.hpp>
#include <kvasir/Runtime/FaultHandler.hpp>
#include <kvasir/StartUp/StartUp.hpp>

#ifndef PORT_FAIL
    #error "PORT_FAIL=<0..7> selects the case"
#endif

namespace {
using Kvasir::Io::makePinLocation;

using LedPin = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin17));
using OutPin = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin6));
using CsPin  = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin7));
using InPin  = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin2));

struct ClockSettings {
    using Tree     = Kvasir::Clocks::Sam::Tree<{.cpu  = Conformance::clockSpeedValue,
                                                .kind = Kvasir::Clocks::Sam::Source::dfllOpenLoop}>;
    using Provides = Tree::Provides;

    static void coreClockInit() { Tree::coreClockInit(); }

    static void peripheryClockInit() {}
};

// the level in the configuring action: high from the first store
struct LedPinConfig {
    static constexpr auto initStepPinConfig = list(makeOutputInitHigh(LedPin{}));
};

// low twice through OUTCLR (one register: legal), and another pin pulled up
struct Pins {
    static constexpr auto initStepPinConfig
      = list(makeOutput(CsPin{}), clear(CsPin{}), action(Kvasir::Io::Action::inputPU, InPin{}));
};

// makeOutput, then the level in an apply() of its own, or after a sequencePoint; two pins' levels in one list
struct Driven {
    static constexpr auto initStepPinConfig = list(makeOutput(OutPin{}));

    static void runtimeInit() {
        apply(set(OutPin{}));
        apply(makeOutput(OutPin{}), Kvasir::Register::sequencePoint, set(OutPin{}));
        apply(set(LedPin{}), clear(OutPin{}), toggle(CsPin{}));
    }
};

#if PORT_FAIL == 1
// makeOutput then set of one pin in one apply(): OUTSET before OUTCLR, the pin ends low
[[maybe_unused]] void refused() { apply(makeOutput(LedPin{}), set(LedPin{})); }

#elif PORT_FAIL == 2
// the same pair in an entry's initStepPinConfig
struct Extra {
    static constexpr auto initStepPinConfig
      = list(makeOutput(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin8)),
             set(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin8)));
};

#elif PORT_FAIL == 3
// the level of a pin another entry configures, in the same phase (one apply() of both)
struct Extra {
    static constexpr auto initStepPinConfig = list(set(OutPin{}));
};

#elif PORT_FAIL == 4
// makeOutputInitHigh and clear: OUTSET then OUTCLR - "high", then low by the address order
[[maybe_unused]] void refused() { apply(makeOutputInitHigh(LedPin{}), clear(LedPin{})); }

#elif PORT_FAIL == 5
// a toggle next to the configured level
[[maybe_unused]] void refused() { apply(makeOutput(LedPin{}), toggle(LedPin{})); }

#elif PORT_FAIL == 6
// a pulled-up input (OUTSET: the pull's direction) cleared in the same list
[[maybe_unused]] void refused() {
    apply(action(Kvasir::Io::Action::inputPU, InPin{}), clear(InPin{}));
}

#elif PORT_FAIL == 7
// input and output of one pin: DIRSET before DIRCLR
[[maybe_unused]] void refused() { apply(makeInput(OutPin{}), makeOutput(OutPin{})); }
#endif
}   // namespace

using Startup = Kvasir::Startup::Startup<ClockSettings,
                                         Conformance::SystickClock,
                                         uc_log::ComBackend<uc_log::Tag::User>,
                                         Kvasir::Boot::Report<>,
                                         Kvasir::Fault::HandlerFor<>,
                                         LedPinConfig,
                                         Pins,
#if PORT_FAIL == 2 || PORT_FAIL == 3
                                         Extra,
#endif
                                         Driven>;

int main() {
    while(true) { Startup::run<Kvasir::Hook::MainLoop>(); }
}

template struct Kvasir::Startup::Start<Startup>;
