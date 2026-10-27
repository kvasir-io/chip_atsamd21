// src/chip/ResetCause.hpp: PM.RCAUSE's causes (DS40001882L 16.8.14) mapped onto the SDK's ResetKind, every
// enumerator. All static_asserts: building is the test. PM.hpp reads the generated registers and is compiled by
// tests/conformance.
#include "ResetCause.hpp"

namespace {
using namespace Kvasir::PM;
using Kvasir::ResetKind;

// the names and values are printed by the firmwares and read by scripts: they stay; `unknown` comes last
static_assert(static_cast<unsigned>(ResetCause::por) == 0
              && static_cast<unsigned>(ResetCause::syst) == 5
              && static_cast<unsigned>(ResetCause::unknown) == 6);

static_assert(toResetKind(ResetCause::por) == ResetKind::powerOn);
static_assert(toResetKind(ResetCause::bod12) == ResetKind::brownOut);
static_assert(toResetKind(ResetCause::bod33) == ResetKind::brownOut);
static_assert(toResetKind(ResetCause::ext) == ResetKind::external);
static_assert(toResetKind(ResetCause::wdt) == ResetKind::watchdogTimeout);
static_assert(toResetKind(ResetCause::syst) == ResetKind::softwareRequest);
static_assert(toResetKind(ResetCause::unknown) == ResetKind::unknown);
}   // namespace

int main() { return 0; }
