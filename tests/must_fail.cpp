// One compile error per MUST_FAIL value; CMakeLists.txt checks that the compiler prints the message
// with the numbers. Without MUST_FAIL the file compiles (and is not built).
#include "ClockLimits.hpp"

namespace L = Kvasir::ClockLimits::D21;
using Kvasir::ClockLimits::Supply;

#if MUST_FAIL == 1
// a hand-written RWS too small for the supply
consteval void f() {
    Kvasir::Nvm::assertWaitStatesFrom<48'000'000, Supply::from1V62, 1, L::WaitStates85C>();
}
#elif MUST_FAIL == 2
// a frequency the table has no row for (the open loop's 49 MHz)
constexpr unsigned ws = L::waitStates<L::DfllOpenLoopMaxHz, Supply::from2V7>();
#elif MUST_FAIL == 3
// the old FDPLL numbers (DIV 128): reference 31 007.75 Hz
constexpr auto c
  = Kvasir::DPLL::assertXoscSetting<L::Fdpll96m, L::DpllWhere, 8'000'000, 128, 3095, 0>();
#elif MUST_FAIL == 4
// nothing reaches 200 MHz
constexpr auto s
  = Kvasir::DPLL::solveXosc<L::Fdpll96m, L::DpllWhere, 8'000'000, 200'000'000, 0, false>();
#elif MUST_FAIL == 5
// a DFLL reference above 33 kHz
consteval void f() {
    Kvasir::ClockLimits::assertInRange<Kvasir::Prescaler::Rational{40'000, 1},
                                       L::Dfll48m.refMin,
                                       L::Dfll48m.refMax,
                                       "DFLL48M reference",
                                       "SAM D21 Table 37-54 f_REF">();
}
#elif MUST_FAIL == 6
// the 105 C rating has no table
constexpr unsigned ws = L::waitStates<48'000'000, Supply::from2V7, L::Rating::c105>();
#elif MUST_FAIL == 7
// a DFLL closed loop outside the tolerance asked for: 1465 x 32768 Hz
consteval void f() {
    constexpr auto loop = Kvasir::DFLL::closedLoop<L::Dfll48m>(32'768, 48'000'000);
    Kvasir::Prescaler::assertInTolerance<loop.achieved,
                                         48'000'000,
                                         Kvasir::Prescaler::Tolerance::ppm(100),
                                         "DFLL48M closed loop">();
}
#endif

int main() { return 0; }
