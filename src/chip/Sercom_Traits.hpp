#pragma once
#include "Variant.hpp"
#include "chip/atsam_common/Sercom_Common.hpp"
#include "kvasir/Io/Types.hpp"

#include <array>

namespace Kvasir { namespace Sercom { namespace Traits {

    struct SercomTraits : SercomTraitsBase<SercomTraits> {
        // {instance, port, pin, pin function (C = 2, D = 3), pad}: SAM D21 datasheet DS40001882L,
        // Tables 7-1 and 7-2 "PORT Function Multiplexing" (md l.1028, l.1123).
        static constexpr std::array pinMuxInfos{
          // Instance 0
          PinInfo{0, 0,  4, 3, 0},
          PinInfo{0, 0,  5, 3, 1},
          PinInfo{0, 0,  6, 3, 2},
          PinInfo{0, 0,  7, 3, 3},
          PinInfo{0, 0,  8, 2, 0},
          PinInfo{0, 0,  9, 2, 1},
          PinInfo{0, 0, 10, 2, 2},
          PinInfo{0, 0, 11, 2, 3},
          // Instance 1
          PinInfo{1, 0, 16, 2, 0},
          PinInfo{1, 0, 17, 2, 1},
          PinInfo{1, 0, 18, 2, 2},
          PinInfo{1, 0, 19, 2, 3},
          PinInfo{1, 0, 30, 3, 2},
          PinInfo{1, 0, 31, 3, 3},
          // Instance 2
          PinInfo{2, 0,  8, 3, 0},
          PinInfo{2, 0,  9, 3, 1},
          PinInfo{2, 0, 10, 3, 2},
          PinInfo{2, 0, 11, 3, 3},
          PinInfo{2, 0, 12, 2, 0},
          PinInfo{2, 0, 13, 2, 1},
          PinInfo{2, 0, 14, 2, 2},
          PinInfo{2, 0, 15, 2, 3},
          // Instance 3
          PinInfo{3, 0, 16, 3, 0},
          PinInfo{3, 0, 17, 3, 1},
          PinInfo{3, 0, 18, 3, 2},
          PinInfo{3, 0, 19, 3, 3},
          PinInfo{3, 0, 20, 3, 2},
          PinInfo{3, 0, 21, 3, 3},
          PinInfo{3, 0, 22, 2, 0},
          PinInfo{3, 0, 23, 2, 1},
          PinInfo{3, 0, 24, 2, 2},
          PinInfo{3, 0, 25, 2, 3},
          // Instance 4
          PinInfo{4, 1,  8, 3, 0},
          PinInfo{4, 1,  9, 3, 1},
          PinInfo{4, 1, 10, 3, 2},
          PinInfo{4, 1, 11, 3, 3},
          PinInfo{4, 0, 12, 3, 0},
          PinInfo{4, 0, 13, 3, 1},
          PinInfo{4, 0, 14, 3, 2},
          PinInfo{4, 0, 15, 3, 3},
          // Instance 5
          PinInfo{5, 0, 20, 2, 2},
          PinInfo{5, 0, 21, 2, 3},
          PinInfo{5, 0, 22, 3, 0},
          PinInfo{5, 0, 23, 3, 1},
          PinInfo{5, 0, 24, 3, 2},
          PinInfo{5, 0, 25, 3, 3},
          PinInfo{5, 1,  2, 3, 0},
          PinInfo{5, 1,  3, 3, 1},
#ifdef KVASIR_CHIP_ATSAMD21G18A
          // What table 7-1 (A variants, column SAMD2xG) has and table 7-2 (L variants) has not:
          // PA00, PA01 and PB22, PB23, which the L parts do not bond out. Everything above is the
          // same in both tables.
          PinInfo{1, 0,  0, 3, 0},
          PinInfo{1, 0,  1, 3, 1},
          PinInfo{5, 1, 22, 3, 2},
          PinInfo{5, 1, 23, 3, 3},
#endif
        };
    };

}}}   // namespace Kvasir::Sercom::Traits
