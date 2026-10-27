#pragma once
// The chip's pin table (PinLocationTraits). Users include the family's pin front, chip/atsam_common/Io.hpp, which
// includes this.

#include "Variant.hpp"
#include "kvasir/Hal/Pins.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/PORT.hpp"

#include <array>
#include <type_traits>

namespace Kvasir { namespace Io {
    template<typename>
    struct PinLocationTraits {
        static constexpr unsigned baseAddress = Kvasir::Peripheral::PORT::Registers<>::baseAddr;
        static constexpr int      portBegin   = 0;
        static constexpr int      portEnd     = 2;
        static constexpr int      pinBegin    = 0;
        static constexpr int      pinEnd      = 32;
        // The 48-pin parts, SAM D21 datasheet DS40001882: table 7-1 (column SAMD2xG) for the A
        // variants, table 7-2 (SAMD21GxL) for the L ones.
#ifdef KVASIR_CHIP_ATSAMD21G18A
        // PA00-PA25, PA27, PA28, PA30, PA31; PB02, PB03, PB08-PB11, PB22, PB23
        static constexpr std::array<std::uint64_t, portEnd - portBegin> bonded{0xDBFF'FFFFULL,
                                                                               0x00C0'0F0CULL};
#else
        // PA02-PA25, PA27, PA28, PA30, PA31; PB00-PB05, PB08-PB11
        static constexpr std::array<std::uint64_t, portEnd - portBegin> bonded{0xDBFF'FFFCULL,
                                                                               0x0000'0F3FULL};
#endif
    };

}}   // namespace Kvasir::Io

namespace Kvasir { namespace Sam {
    // The PORT registers of one pin group (port), for chip_atsam_common Io.hpp. The D21's SVD has the group registers
    // as dims (DIRCLR<Group>, ...) and the per-pin ones split by group (PINCFG0<Pin>/PINCFG1<Pin>, PMUX0/PMUX1 by
    // pin pair; DS40001882L 23.7, PINCFGy 0x40 + y and PMUXn 0x30 + n per group, group stride 0x80).
    template<int Port>
    struct PortGroup {
        static_assert(Port == 0 || Port == 1,
                      "the D21 has PORT groups 0 (PA) and 1 (PB)");
        using Regs = Kvasir::Peripheral::PORT::Registers<>;

        static constexpr auto dirclr = Regs::DIRCLR<Port>::dirclr;
        static constexpr auto dirset = Regs::DIRSET<Port>::dirset;
        static constexpr auto outclr = Regs::OUTCLR<Port>::outclr;
        static constexpr auto outset = Regs::OUTSET<Port>::outset;
        static constexpr auto outtgl = Regs::OUTTGL<Port>::outtgl;
        static constexpr auto in     = Regs::IN<Port>::in;

        template<int Pin>
        using Pincfg = std::conditional_t<Port == 0, Regs::PINCFG0<Pin>, Regs::PINCFG1<Pin>>;
        template<int Pin>
        using Pmux = std::conditional_t<Port == 0, Regs::PMUX0<Pin / 2>, Regs::PMUX1<Pin / 2>>;
    };
}}   // namespace Kvasir::Sam
