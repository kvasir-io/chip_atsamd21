#pragma once

#include "Variant.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/PORT.hpp"

#include <array>

namespace Kvasir { namespace Io {
    template<typename>
    struct PinLocationTraits {
        static constexpr unsigned baseAddress = Kvasir::Peripheral::PORT::Registers<>::baseAddr;
        static constexpr int      portBegin   = 0;
        static constexpr int      portEnd     = 2;
        static constexpr int      pinBegin    = 0;
        static constexpr int      pinEnd      = 32;
        static constexpr int      ListEndIndicator = 255;
        // The 48-pin parts, SAM D21 datasheet DS40001882: table 7-1 (column SAMD2xG) for the A
        // variants, table 7-2 (SAMD21GxL) for the L ones.
#if defined(KVASIR_CHIP_ATSAMD21G18A)
        // PA00-PA25, PA27, PA28, PA30, PA31; PB02, PB03, PB08-PB11, PB22, PB23
        static constexpr std::array<std::array<int, pinEnd - pinBegin>, portEnd - portBegin>
          PinsDisabled{
            {{{26, 29, ListEndIndicator}}, {{0,  1,  4,  5,  6,
                                             7,  12, 13, 14, 15,
                                             16, 17, 18, 19, 20,
                                             21, 24, 25, 26, 27,
                                             28, 29, 30, 31, ListEndIndicator}}}
        };
#else
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
               ListEndIndicator}}}
        };
#endif
    };

}}   // namespace Kvasir::Io

#include "atsam_common/Io.hpp"
