#pragma once
// The bounds of the SAM D21 package's register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Under the default wait policy (Unbounded) every site is the loop it was.
#include "ClockLimits.hpp"
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

namespace Kvasir::Chip {
// f_CPU max 48 MHz (Table 37-7): Bound::Microseconds converts at this, a slower clock only lengthens the real time
inline constexpr std::uint64_t CpuHzCeiling = ClockLimits::D21::CpuMax;

template<std::uint64_t Us>
using MicrosecondsBound = Kvasir::Register::Bound::Microseconds<Us, CpuHzCeiling>;

// XOSC.STARTUP n: 2^n OSCULP32K cycles (DS40001882L 17.8.5, md l.8456-8487); 15 = 32768 cycles, ~1 s
template<std::uint16_t Startup>
using XoscReadyBound
  = MicrosecondsBound<(std::uint64_t{1} << Startup) * 1'000'000 / 32'768 + 1'000>;
// FDPLL96M tLOCK max 2 ms at f_IN 32 kHz, 50 us at 2 MHz (Table 37-58, md l.42239-42256)
using DpllLockBound = MicrosecondsBound<2'000>;
// DFLL48M closed-loop tLOCK max 500 us (Tables 37-53/37-54, md l.42146-42165), x10
using DfllLockBound = MicrosecondsBound<5'000>;
// PCLKSR.DFLLRDY after a DFLL register write: the synchronisation, 5 P_GCLK + 2 P_APB < D < 6 P_GCLK + 3 P_APB
// (14.3.1.8, md l.4417-4423); at a 32 kHz GCLK 190 us
using DfllSyncBound = MicrosecondsBound<10'000>;
}   // namespace Kvasir::Chip
