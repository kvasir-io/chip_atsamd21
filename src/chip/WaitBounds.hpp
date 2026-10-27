#pragma once
// The bounds of the SAM D21 package's register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Under the default wait policy (Unbounded) every site is the loop it was.
#include "ClockLimits.hpp"
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

// f_CPU max 48 MHz (Table 37-7): the SDK's MicrosecondsBound converts at this, a slower clock only lengthens the real
// time. Specialised before any bound below names it.
template<>
struct Kvasir::Register::CpuHzCeiling<Kvasir::Tag::User> {
    static constexpr std::uint64_t value = Kvasir::ClockLimits::D21::cpuMax;
};

namespace Kvasir::Chip {
// FDPLL96M tLOCK max 2 ms at f_IN 32 kHz, 50 us at 2 MHz (Table 37-58, md l.42239-42256)
using DpllLockBound = Kvasir::Register::MicrosecondsBound<2'000>;
// DFLL48M closed-loop tLOCK max 500 us (Tables 37-53/37-54, md l.42146-42165), x10
using DfllLockBound = Kvasir::Register::MicrosecondsBound<5'000>;
// PCLKSR.DFLLRDY after a DFLL register write: the synchronisation, 5 P_GCLK + 2 P_APB < D < 6 P_GCLK + 3 P_APB
// (14.3.1.8, md l.4417-4423); at a 32 kHz GCLK 190 us
using DfllSyncBound = Kvasir::Register::MicrosecondsBound<10'000>;
// PCLKSR.OSC8MRDY after OSC8M.ENABLE: tSTARTUP max 3.3 us (Table 37-57, md l.42226), x30
using Osc8mReadyBound = Kvasir::Register::MicrosecondsBound<100>;
}   // namespace Kvasir::Chip

// the shared SAM drivers' bounds (Sam::SyncBound, ...): they need the ceiling above
#include "atsam_common/SamWaitBounds.hpp"
