#pragma once
// The SAM D21's NVM controller for the shared driver (atsam_common/NVMCTRL.hpp): the D2x/C2x command set
// (atsam_common/NvmCommands.hpp) with this chip's errata, bounds and geometry.

#include "ClockLimits.hpp"
#include "Variant.hpp"
#include "WaitBounds.hpp"
#include "atsam_common/NvmCommands.hpp"
#include "kvasir/Register/Memory.hpp"
#include "peripherals/NVMCTRL.hpp"

#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace NVMCTRL {
    struct Errata {
        using NVM = Kvasir::Peripheral::NVMCTRL::Registers<>;

        // D21 errata 1.11.3 (DS80000760M, rev E): READY is not cleared by RWWEEER, and a second RWWEEER aborts the
        // first. Workaround: a dummy write to the page buffer right before the command. Done on every revision, no
        // revision check: it only loads one word of the buffer (the next write starts with PBC) and sets ADDR to this
        // row (22.8.8: "automatically updated when writing to the page buffer"), the row the erase is for anyway.
        template<typename T>
        static void prepareEraseRww(T addr) {
            Kvasir::Memory::store<std::uint32_t>(static_cast<std::uintptr_t>(addr), 0xFFFF'FFFFU);
        }

        // D21 errata 1.11.4 (DS80000760M): an NVM command on the RWWEE array with the NVM cache on "can result in
        // incorrect data fetch afterward"; the cache is off around every command (CTRLB.CACHEDIS, 22.8.2; a field write:
        // CTRLB keeps MANW and RWS).
        static void enableCache() { apply(clear(NVM::CTRLB::cachedis)); }

        static void disableCache() { apply(set(NVM::CTRLB::cachedis)); }
    };

    struct Traits : Sam::NvmCommandsDC<Kvasir::Peripheral::NVMCTRL::Registers<>, Errata> {
        // The 256 KB part (G18A) has no RWWEE array: "not available on 256 KB devices" (DS40001882L Features, md l.17),
        // "Only available on device variant B, C, D, and L" (10.1, md l.1494). An RWWEE store on it is then refused
        // at compile time (NVMCTRL.hpp, ChipTraits.hpp) instead of by PARAM.RWWEEP = 0 at run time.
        static constexpr bool hasRwweeArray
          = Kvasir::Chip::part != Kvasir::Chip::Part::atsamd21g18a;

        // tFRE (row erase) max 6 ms, tFPP (page programming) max 2.5 ms (DS40001882L Tables 37-45/37-46, md
        // l.41964-41975); x2, at the highest CPU clock (a slower one only lengthens the real time)
        using EraseBound = Kvasir::Register::MicrosecondsBound<12'000>;
        using WriteBound = Kvasir::Register::MicrosecondsBound<5'000>;

        static constexpr std::size_t pageSize    = 64;
        static constexpr std::size_t pagesPerRow = 4;
    };
}}   // namespace Kvasir::NVMCTRL
