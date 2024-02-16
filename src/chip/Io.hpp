#pragma once

#include "kvasir/Io/Io.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/PORT.hpp"

#include <array>
namespace Kvasir { namespace Io {
    template<>
    struct PinLocationTraits<void> {
        static constexpr unsigned baseAddress = Kvasir::Peripheral::PORT::Registers<>::baseAddr;
        static constexpr int      portBegin   = 0;
        static constexpr int      portEnd     = 2;
        static constexpr int      pinBegin    = 0;
        static constexpr int      pinEnd      = 32;
        static constexpr int      ListEndIndicator = 255;
        static constexpr std::array<std::array<int, pinEnd - pinBegin>, portEnd - portBegin>
          PinsDisabled{
            {{{0, 1, 26, 29, ListEndIndicator}},
             {{6,
               7,
               12,
               13,
               14,
               15,
               16,
               17,
               18,
               19,
               20,
               21,
               22,
               23,
               24,
               25,
               26,
               27,
               28,
               29,
               30,
               31,
               ListEndIndicator}}}};
    };

}}   // namespace Kvasir::Io

#include "atsam_common/Io.hpp"
