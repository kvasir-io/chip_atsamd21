// SAM pin ownership in a Startup list as must-fail cases (kvasir_conformance_must_fail, CMakeLists.txt): the PMUXEN
// write of a pin's PINCFG in an entry's initStepPinConfig provides `Io::PinResource<port, pin>` (chip_atsam_common
// Io.hpp). PINS_FAIL=0 must compile: three entries configure four pins of PORTA (two pins of one port in two entries),
// a fourth claims a pin it does not configure (Io::PinClaims). Each of 1..2 adds one entry that configures a pin a
// second time and must fail with the pin's name (the case list in CMakeLists.txt).

#include "kvasir_config.hpp"
//
#include <chip/ClockTree.hpp>
#include <chip/chip.hpp>
#include <kvasir/Io/Claims.hpp>
#include <kvasir/Runtime/BootReport.hpp>
#include <kvasir/Runtime/FaultHandler.hpp>
#include <kvasir/StartUp/StartUp.hpp>

#ifndef PINS_FAIL
    #error "PINS_FAIL=<0..2> selects the case"
#endif

namespace {
using Kvasir::Io::makePinLocation;

using LedPin = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin17));
using OutPin = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin6));
using InPin  = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin2));
using MuxPin = decltype(makePinLocation(Kvasir::Io::portA, Kvasir::Io::pin8));

struct ClockSettings {
    using Tree     = Kvasir::Clocks::Sam::Tree<{.cpu  = Conformance::clockSpeedValue,
                                                .kind = Kvasir::Clocks::Sam::Source::dfllOpenLoop}>;
    using Provides = Tree::Provides;

    static void coreClockInit() { Tree::coreClockInit(); }

    static void peripheryClockInit() {}
};

struct LedPinConfig {
    static constexpr auto initStepPinConfig = list(makeOutput(LedPin{}));
};

// two pins of PORTA in one entry, beside the LED's: different pins of one port in two entries build
struct Pins {
    static constexpr auto initStepPinConfig
      = list(makeOutput(OutPin{}), action(Kvasir::Io::Action::inputPU, InPin{}));
};

// a peripheral function on a pin (PMUX + PINCFG.PMUXEN)
struct Mux {
    static constexpr auto initStepPinConfig
      = list(action(Kvasir::Io::Action::PinFunction<2>{}, MuxPin{}));
};

// reads InPin, configured by Pins: a claim, not a second configuration
struct Watcher {
    using Claims = Kvasir::Io::PinClaims<InPin>;
};

#if PINS_FAIL == 1   // the LED pin configured by a second entry
struct Extra {
    static constexpr auto initStepPinConfig = list(makeInput(LedPin{}));
};
#elif PINS_FAIL == 2   // an output on the pin Mux gave a peripheral function
struct Extra {
    static constexpr auto initStepPinConfig = list(makeOutput(MuxPin{}));
};
#endif
}   // namespace

using Startup = Kvasir::Startup::Startup<ClockSettings,
                                         Conformance::SystickClock,
                                         uc_log::ComBackend<uc_log::Tag::User>,
                                         Kvasir::Boot::Report<>,
                                         Kvasir::Fault::HandlerFor<>,
                                         LedPinConfig,
                                         Pins,
                                         Mux,
#if PINS_FAIL != 0
                                         Extra,
#endif
                                         Watcher>;

int main() {
    while(true) { Startup::run<Kvasir::Hook::MainLoop>(); }
}

template struct Kvasir::Startup::Start<Startup>;
