// The conformance tree's main TU (Kvasir_SDK tests/conformance/conformance.cmake MAIN_TU): an image's main TU in the
// order a main may choose - kvasir/StartUp/StartUp.hpp FIRST, the chip after it - with a Startup instantiated, so every
// late hook (FirstInitStep, the fault handler's interrupt traits, the NVIC clear-all's traits, the chip's entries) is
// compiled behind the SDK's header. Linked with a second TU (chip/chip.hpp + chip/drivers.hpp) into one object, and
// with a copy of itself, which must fail on the one-TU rule's guard. Never run.
// clang-format off: the SDK's Startup header first, on purpose
#include <kvasir/StartUp/StartUp.hpp>
// clang-format on

#include <chip/ClockTree.hpp>
#include <chip/chip.hpp>
#include <kvasir/Runtime/BootReport.hpp>   // after the chip: its reset cause by name (until step 140)
#include <kvasir/Runtime/FaultHandler.hpp>

namespace {
// DFLL48M in open loop: no crystal, no USB (chip/ClockTree.hpp)
struct ClockSettings {
    using Tree     = Kvasir::Clocks::Sam::Tree<{.cpu  = Conformance::clockSpeedValue,
                                                .kind = Kvasir::Clocks::Sam::Source::dfllOpenLoop}>;
    using Provides = Tree::Provides;

    static void coreClockInit() { Tree::coreClockInit(); }

    static void peripheryClockInit() {}
};
}   // namespace

using Startup = Kvasir::Startup::Startup<ClockSettings,
                                         Conformance::SystickClock,
                                         uc_log::ComBackend<uc_log::Tag::User>,
                                         Kvasir::Boot::Report<>,
                                         Kvasir::Fault::HandlerFor<>>;

int main() {
    while(true) { Startup::run<Kvasir::Hook::MainLoop>(); }
}

template struct Kvasir::Startup::Start<Startup>;
