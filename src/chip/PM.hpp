#pragma once

#include "Blocks.hpp"
#include "ResetCause.hpp"
#include "atsam_common/ResetCauseDecode.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Port/ResetKind.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/PM.hpp"

#include <array>

namespace Kvasir { namespace PM {
    // The clock gates (enable, disable, isEnabled) are chip/Blocks.hpp over atsam_common/ClockGate.hpp.

    /// The latest reset's cause (PM.RCAUSE, DS40001882L 16.8.14; the causes in ResetCause.hpp), one load.
    inline ResetCause resetCause() {
        using R         = Kvasir::Peripheral::PM::Registers<>::RCAUSE;
        auto const bits = apply(read(R::por, R::bod12, R::bod33, R::ext, R::wdt, R::syst));
        return Sam::oneCauseOf(bits,
                               std::array{ResetCause::por,
                                          ResetCause::bod12,
                                          ResetCause::bod33,
                                          ResetCause::ext,
                                          ResetCause::wdt,
                                          ResetCause::syst},
                               ResetCause::unknown);
    }

    /// resetCause() mapped onto the SDK's ResetKind (ResetCause.hpp toResetKind), for the boot guard.
    inline ResetKind resetKind() { return toResetKind(resetCause()); }
}}   // namespace Kvasir::PM

/// resetCause() as the port's hook (kvasir/Port/Hooks.hpp), for the boot report:
/// Port::LateT<Port::ResetCause<Tag::User>, Dep>{}().
template<typename... Ts>
struct Kvasir::Port::ResetCause<Kvasir::Tag::User, Ts...> {
    Kvasir::PM::ResetCause operator()() const { return Kvasir::PM::resetCause(); }
};
