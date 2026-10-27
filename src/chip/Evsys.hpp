#pragma once
#include "chip/PM.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/EVSYS.hpp"

namespace Kvasir { namespace EVSYS {
    struct EvsysBase {
        using Regs      = Kvasir::Peripheral::EVSYS::Registers<>;
        using Path      = Regs::CHANNEL::PATHVal;
        using Edge      = Regs::CHANNEL::EDGSELVal;
        using User      = Regs::USER::USERVal;
        using Generator = Regs::CHANNEL::EVGENVal;

        enum class Channel {
            ch0  = 0,
            ch1  = 1,
            ch2  = 2,
            ch3  = 3,
            ch4  = 4,
            ch5  = 5,
            ch6  = 6,
            ch7  = 7,
            ch8  = 8,
            ch9  = 9,
            ch10 = 10,
            ch11 = 11
        };

        static constexpr auto powerClockEnable = list(PM::Enable<Regs::baseAddr>::action{});
        static constexpr auto initStepPeripheryConfig
          = list(set(Regs::CTRL::gclkreq), clear(Regs::CTRL::swrst));

        template<Channel   Channel,
                 Generator Generator,
                 User      User,
                 Path      Path = Path::synchronous,
                 Edge      Edge = Edge::rising_edge>
        static auto setupEventChannel() -> decltype(list(
          write(Regs::USER::channel,
                Register::value<static_cast<unsigned>(Channel) + 1>()),
          write(Register::FieldValue<typename decltype(Regs::USER::user)::type,
                                     User>{}),
          Register::sequencePoint,
          write(Regs::CHANNEL::channel,
                Register::value<static_cast<unsigned>(Channel)>()),
          clear(Regs::CHANNEL::swevt),
          write(Register::FieldValue<typename decltype(Regs::CHANNEL::evgen)::type,
                                     Generator>{}),
          write(Register::FieldValue<typename decltype(Regs::CHANNEL::path)::type,
                                     Path>{}),
          write(Register::FieldValue<typename decltype(Regs::CHANNEL::edgsel)::type,
                                     Edge>{}))) {
            return {};
        }
    };

}}   // namespace Kvasir::EVSYS
