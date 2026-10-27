#pragma once

#include "kvasir/Port/ResetKind.hpp"

#include <cstdint>

// The reset causes of PM.RCAUSE (SAM D21/DA1 data sheet DS40001882L 16.8.14) and their mapping onto the SDK's
// ResetKind, without registers (host-tested in tests/reset_cause_test.cpp); PM.hpp reads the register.
namespace Kvasir { namespace PM {
    /// `unknown`: no bit or more than one bit set (atsam_common/ResetCauseDecode.hpp). The names are printed by the
    /// boot report.
    enum class ResetCause : std::uint8_t { por, bod12, bod33, ext, wdt, syst, unknown };

    /// resetCause() as the SDK's boot guard sees it (kvasir/Port/ResetKind.hpp). A debugger's reset is SYST like a
    /// software request: without a record the guard counts neither.
    [[nodiscard]] constexpr ResetKind toResetKind(ResetCause c) {
        switch(c) {
        case ResetCause::por:     return ResetKind::powerOn;
        case ResetCause::bod12:
        case ResetCause::bod33:   return ResetKind::brownOut;
        case ResetCause::ext:     return ResetKind::external;
        case ResetCause::wdt:     return ResetKind::watchdogTimeout;
        case ResetCause::syst:    return ResetKind::softwareRequest;
        case ResetCause::unknown: return ResetKind::unknown;
        }
        return ResetKind::unknown;
    }
}}   // namespace Kvasir::PM
