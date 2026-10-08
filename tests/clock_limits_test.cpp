// src/chip/ClockLimits.hpp (SAM D21) through chip_atsam_common's ClockSolver.hpp, as the firmwares
// use it - wait-state tables at their edges, the DFLL48M multiplier. Most checks are static_asserts.
#include "ClockLimits.hpp"

#include <cstdio>

namespace {
using namespace Kvasir;
namespace L = ClockLimits::D21;
using ClockLimits::Supply;
using L::Rating;

// 8 MHz crystal -> 48 MHz: DCO 48 MHz (the lowest), the reference 1 MHz: 2 MHz (DIV 1) sits on
// the f_IN limit, 1.333 MHz (DIV 2) is not a whole number of hertz. The numbers
// firmwares use (DIV 3, LDR 47, GCLK0 / 1).
constexpr auto d8 = DPLL::fromXosc<L::Fdpll96m>(8'000'000, 48'000'000);
static_assert(d8.found && d8.div == 3 && d8.ldr == 47 && d8.ldrFrac == 0 && d8.presc == 0
              && d8.gclkDiv == 1);
// the value, not the representation: Rational is not reduced
static_assert(d8.achieved.num == 48'000'000 * d8.achieved.den);
static_assert(d8.dco.num == 48'000'000 * d8.dco.den && d8.ref.num == 1'000'000 * d8.ref.den);
// what the hand-written settings make, checked the other way round
static_assert(DPLL::checkXosc<L::Fdpll96m>(8'000'000,
                                           3,
                                           47,
                                           0)
                .ok());

// the old DIV 128 setting is out of range: DIV 128 gives 31 007 Hz
constexpr auto old = DPLL::checkXosc<L::Fdpll96m>(8'000'000, 128, 3095, 0, 0, 2);
static_assert(!old.refInRange && old.dcoInRange && old.fieldsFit && !old.ok());
static_assert(old.out.num == 48'000'000 * old.out.den);

// 12 MHz: 2 MHz (DIV 2) on the limit, 1.5 MHz (DIV 3) x 32; 16 MHz: 2 MHz (DIV 3) on the limit,
// 1.6 MHz (DIV 4) x 30
constexpr auto d12 = DPLL::fromXosc<L::Fdpll96m>(12'000'000, 48'000'000);
static_assert(d12.found && d12.div == 3 && d12.ldr == 31 && d12.gclkDiv == 1);
constexpr auto d16 = DPLL::fromXosc<L::Fdpll96m>(16'000'000, 48'000'000);
static_assert(d16.found && d16.div == 4 && d16.ldr == 29 && d16.gclkDiv == 1);

// 8 -> 96 MHz: the DCO at its maximum, 1 MHz x 96. Above 64 MHz: errata 1.3.2 (revisions A-D)
constexpr auto d96 = DPLL::fromXosc<L::Fdpll96m>(8'000'000, 96'000'000);
static_assert(d96.found && d96.div == 3 && d96.ldr == 95 && d96.gclkDiv == 1);
static_assert(Prescaler::above(d96.dco,
                               L::DcoMaxBelow0C));
// 24 MHz is below the DCO's 48 MHz: 48 MHz and the generator divides by 2
constexpr auto d24 = DPLL::fromXosc<L::Fdpll96m>(8'000'000, 24'000'000);
static_assert(d24.found && d24.div == 3 && d24.ldr == 47 && d24.gclkDiv == 2);

// XOSC32K straight in: 48 MHz / 32768 = 1464.84 - not exact in integer mode, and not within 1/16
// steps either (1464 + 14/16 = 48.0010 MHz)
static_assert(!DPLL::fromReference<L::Fdpll96m>(32'768,
                                                48'000'000)
                 .found);
constexpr auto x32
  = DPLL::fromReference<L::Fdpll96m>(32'768, 48'000'000, Prescaler::Tolerance::ppm(100), true);
static_assert(x32.found && x32.ldr == 1463 && x32.ldrFrac == 14);
// ... and in integer mode with a 1000 ppm tolerance: 1465 x 32768 = 48.005 MHz (+107 ppm)
constexpr auto x32i
  = DPLL::fromReference<L::Fdpll96m>(32'768, 48'000'000, Prescaler::Tolerance::ppm(1000));
static_assert(x32i.found && x32i.ldr == 1464 && x32i.ldrFrac == 0);
// 32.768 kHz -> 96 MHz: the nearest ratio, 2930, gives 96.010 MHz - above f_OUT's 96 MHz
static_assert(!DPLL::fromReference<L::Fdpll96m>(32'768,
                                                96'000'000,
                                                Prescaler::Tolerance::ppm(1000))
                 .found);

// a reference below f_IN is refused outright
static_assert(!DPLL::fromReference<L::Fdpll96m>(31'000,
                                                48'000'000,
                                                Prescaler::Tolerance::ppm(100'000),
                                                true)
                 .found);
// nothing reaches 200 MHz
static_assert(!DPLL::fromXosc<L::Fdpll96m>(8'000'000,
                                           200'000'000)
                 .found);

// the tie-break rules one by one
namespace DD = DPLL::detail;

constexpr DPLL::Setting mk(std::uint64_t refNum,
                           std::uint64_t refDen,
                           std::uint64_t dco,
                           std::uint32_t frac = 0) {
    return {
      0,
      0,
      frac,
      0,
      1,
      {   dco,      1},
      {refNum, refDen},
      {   dco,      1},
      true
    };
}

// integer mode first, whatever the DCO
static_assert(DD::better<L::Fdpll96m>(mk(1'000'000,
                                         1,
                                         96'000'000),
                                      mk(1'000'000,
                                         1,
                                         48'000'000,
                                         3),
                                      48'000'000));
// the lower DCO next
static_assert(DD::better<L::Fdpll96m>(mk(1'000'000,
                                         1,
                                         48'000'000),
                                      mk(2'000'000,
                                         1,
                                         96'000'000),
                                      48'000'000));
// a reference inside the range beats one on its limit (2 MHz), and one on the lower limit too
static_assert(DD::better<L::Fdpll96m>(mk(500'000,
                                         1,
                                         48'000'000),
                                      mk(2'000'000,
                                         1,
                                         48'000'000),
                                      48'000'000));
static_assert(DD::better<L::Fdpll96m>(mk(40'000,
                                         1,
                                         48'000'000),
                                      mk(32'000,
                                         1,
                                         48'000'000),
                                      48'000'000));
// a whole number of hertz beats a higher fraction
static_assert(DD::better<L::Fdpll96m>(mk(1'000'000,
                                         1,
                                         48'000'000),
                                      mk(8'000'000,
                                         6,
                                         48'000'000),
                                      48'000'000));
// then the higher reference; equal values in other representations are a tie, not "better"
static_assert(DD::better<L::Fdpll96m>(mk(1'000'000,
                                         1,
                                         48'000'000),
                                      mk(500'000,
                                         1,
                                         48'000'000),
                                      48'000'000));
static_assert(!DD::better<L::Fdpll96m>(mk(2'000'000,
                                          2,
                                          48'000'000),
                                       mk(1'000'000,
                                          1,
                                          48'000'000),
                                       48'000'000));

// wait states at every row edge (Table 37-42)
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from2V7,
                              24'000'000)
              == 0);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from2V7,
                              24'000'001)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from2V7,
                              48'000'000)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from2V7,
                              48'000'001)
              == Nvm::NoWaitStateCount);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              14'000'000)
              == 0);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              14'000'001)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              28'000'000)
              == 1);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              28'000'001)
              == 2);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              42'000'000)
              == 2);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              42'000'001)
              == 3);
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from1V62,
                              48'000'000)
              == 3);
// a supply the table does not have
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from4V5,
                              1'000'000)
              == Nvm::NoWaitStateCount);
// the template the firmwares use, per rating
static_assert(L::waitStates<48'000'000,
                            Supply::from2V7>()
              == 1);
static_assert(L::waitStates<48'000'000,
                            Supply::from1V62>()
              == 3);
static_assert(L::waitStates<40'000'000,
                            Supply::from2V7,
                            Rating::c125>()
              == 1);
static_assert(L::waitStates<40'000'000,
                            Supply::from1V62,
                            Rating::c125>()
              == 2);
static_assert(L::waitStates<40'000'000,
                            Supply::from2V7,
                            Rating::aecQ100VariantA>()
              == 1);
static_assert(L::waitStates<48'000'000,
                            Supply::from2V7,
                            Rating::aecQ100VariantBD>()
              == 1);
// the open loop's worst case is above every row
static_assert(Nvm::waitStates(L::WaitStates85C,
                              Supply::from2V7,
                              L::DfllOpenLoopMaxHz)
              == Nvm::NoWaitStateCount);

// DFLL48M closed loop on 32 768 Hz: MUL 1465, 48 005 120 Hz; a 40 kHz reference is out of range
constexpr auto dfll = DFLL::closedLoop<L::Dfll48m>(32'768, 48'000'000);
static_assert(dfll.mul == 1465 && dfll.achieved.num == 48'005'120 * dfll.achieved.den
              && dfll.refInRange && dfll.mulFits);
static_assert(!DFLL::closedLoop<L::Dfll48m>(40'000,
                                            48'000'000)
                 .refInRange);
static_assert(!DFLL::closedLoop<L::Dfll48m>(732,
                                            48'000'000)
                 .mulFits,
              "65574 needs 17 bits");
// the step limits: 50 % of COARSE (6 bits) and FINE (10 bits)
static_assert(DFLL::MaxCoarseStep<L::Dfll48m> == 31 && DFLL::MaxFineStep<L::Dfll48m> == 511);

// GCLK division maxima (15.8.5): 512, 131072, 64, 512
static_assert(L::gclkMaxDivision(0) == 512 && L::gclkMaxDivision(1) == 131072
              && L::gclkMaxDivision(2) == 64 && L::gclkMaxDivision(8) == 512);

// the messages, as text (the must-fail tests check that the compiler prints them)
static_assert(
  Nvm::WaitStateMessage<48'000'000,
                        Supply::from1V62,
                        1,
                        L::WaitStates85C>{}
    .view()
  == "flash wait states: 48000000 Hz at 1.62-2.7 V needs 3 (SAM D21 Table 37-42), RWS is 1");
static_assert(ClockLimits::RangeMessage<Prescaler::Rational{8'000'000, 258},
                                        32'000,
                                        2'000'000,
                                        "FDPLL reference",
                                        L::DpllWhere>{}
                .view()
              == "FDPLL reference: wanted 32000..2000000 Hz, got 31007.8 Hz (SAM D21 FDPLL96M, "
                 "Tables 37-58/59/60)");

// the asserted forms compile for good numbers
static_assert(DPLL::solveXosc<L::Fdpll96m,
                              L::DpllWhere,
                              8'000'000,
                              48'000'000,
                              0,
                              false>()
                .ldr
              == 47);
static_assert(DPLL::assertXoscSetting<L::Fdpll96m,
                                      L::DpllWhere,
                                      8'000'000,
                                      3,
                                      47,
                                      0>()
                .ok());
}   // namespace

int main() {
    std::puts("clock solver (SAM D21): every check is a static_assert");
    return 0;
}
