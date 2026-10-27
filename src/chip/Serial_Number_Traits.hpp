#pragma once
#include <array>
#include <cstdint>

namespace Kvasir { namespace SerialNumber { namespace Traits {

    /// SAM D21/DA1 data sheet (DS40001882L) 10.3.3 "Serial Number" (md l.1718-1724), words 0 to 3.
    static constexpr std::array<std::uint32_t, 4> addresses{0x0080A00C,
                                                            0x0080A040,
                                                            0x0080A044,
                                                            0x0080A048};

}}}   // namespace Kvasir::SerialNumber::Traits
