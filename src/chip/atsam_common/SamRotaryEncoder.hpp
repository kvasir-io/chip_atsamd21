#pragma once
#include "EIC.hpp"
#include "kvasir/Devices/RotaryEncoder.hpp"

namespace Kvasir {
template<typename Clock, typename PinA, typename PinB, typename ValueType, typename Config_>
struct SamRotaryEncoder : Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_> {
    using Base = Kvasir::RotaryEncoder<Clock, PinA, PinB, ValueType, Config_>;
    using Base::cnt;

    static constexpr auto initStepPinConfig = list(makeInput(PinB{}));

    struct EicConfig {
        static constexpr auto pin         = PinA{};
        static constexpr auto pull        = Kvasir::Io::PullConfiguration::PullUp;
        static constexpr auto type        = Kvasir::EIC::InterruptType::EdgeBoth;
        static constexpr auto filter      = true;
        static constexpr auto callback    = Base::edgeCallback;
        static constexpr auto enableEvent = false;
    };
};
}   // namespace Kvasir
