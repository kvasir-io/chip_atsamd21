#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

namespace Kvasir { namespace DSU {
    template<unsigned baseAddr = 0x41002000>
    struct Registers {
        static constexpr unsigned BaseAddr = baseAddr;

        struct STATUSB {
            using Addr = Register::Address<baseAddr + 0x02, 0xFF, 0x00, unsigned char>;
            /// Protected
            static constexpr Register::
              FieldLocation<Addr, Register::maskFromRange(0, 0), Register::ReadOnlyAccess, unsigned>
                prot{};
            /// Debugger Present
            static constexpr Register::
              FieldLocation<Addr, Register::maskFromRange(1, 1), Register::ReadOnlyAccess, bool>
                dbgpres{};
            /// Debug Communication Channel 0 Dirty
            static constexpr Register::
              FieldLocation<Addr, Register::maskFromRange(2, 2), Register::ReadOnlyAccess, unsigned>
                dccd0{};
            /// Debug Communication Channel 1 Dirty
            static constexpr Register::
              FieldLocation<Addr, Register::maskFromRange(3, 3), Register::ReadOnlyAccess, unsigned>
                dccd1{};
            /// Hot-Plugging Enable
            static constexpr Register::
              FieldLocation<Addr, Register::maskFromRange(4, 4), Register::ReadOnlyAccess, unsigned>
                hpe{};
        };
    };
}}   // namespace Kvasir::DSU
