// The user-row fields (src/chip/Fuse_Traits.hpp) against SAM D21 DS40001882L 10.3.1 Table 10-7: a row of the table's
// defaults (reserved bits 0, BOD12 24:17 = 0x70, WDT Window 0xB as on silicon, errata DS80000760M 1.5.4) decodes to
// those defaults, and the fields combined into a zeroed row give the row back. Building is the test (constexpr).
#include "Fuse_Traits.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace {
using Row = std::array<std::byte, 8>;
using namespace Kvasir::Fuses::Traits;

constexpr Row row(std::uint32_t word0,
                  std::uint32_t word1) {
    return std::bit_cast<Row>(std::array<std::uint32_t, 2>{word0, word1});
}

// BOOTPROT 7, EEPROM 7 << 4, BOD33 level 7 << 8, enable bit 14, action 1 << 15, BOD12 0x70 << 17, WDT period
// 0xB << 27, Window bit 0 at 31 -> 0xD8E0C777; Window bits 3:1 = 5 at 32, EWOFFSET 0xB << 3, LOCK 0xFFFF << 16
// -> 0xFFFF005D
constexpr Row Defaults = row(0xD8E0'C777U, 0xFFFF'005DU);

constexpr Bod33 bod{Defaults};
static_assert(bod.action.asValue() == 1 && bod.enable.asValue() == 1
              && bod.hysteresis.asValue() == 0 && bod.level.asValue() == 7);
constexpr Wdt wdt{Defaults};
static_assert(wdt.enable.asValue() == 0 && wdt.alwaysOn.asValue() == 0
              && wdt.windowEnable.asValue() == 0 && wdt.period.asValue() == 0xB
              && wdt.window.asValue() == 0xB && wdt.offset.asValue() == 0xB);
constexpr Nvm nvm{Defaults};
static_assert(nvm.bootprot.asValue() == 7 && nvm.eeprom.asValue() == 7
              && nvm.lock.asValue() == 0xFFFF);

// every field combined into a row that holds only the BOD12 bits gives the defaults back
static_assert([] {
    Row r = row(0x70U << 17U, 0);
    bod.action.combineInto(r);
    bod.enable.combineInto(r);
    bod.hysteresis.combineInto(r);
    bod.level.combineInto(r);
    wdt.enable.combineInto(r);
    wdt.alwaysOn.combineInto(r);
    wdt.windowEnable.combineInto(r);
    wdt.period.combineInto(r);
    wdt.window.combineInto(r);
    wdt.offset.combineInto(r);
    nvm.bootprot.combineInto(r);
    nvm.eeprom.combineInto(r);
    nvm.lock.combineInto(r);
    return r == Defaults;
}());

// a field write leaves the BOD12 bits (24:17, 41) and the reserved bits as they were
static_assert([] {
    Row r = row(0xFFFF'FFFFU, 0xFFFF'FFFFU);
    Wdt{Wdt::Enable::disabled,
        Wdt::AlwaysOn::disabled,
        Wdt::WindowEnable::disabled,
        Wdt::Period{0},
        Wdt::Window{0},
        Wdt::Offset{0}}
      .window.combineInto(r);
    return r == row(0x7FFF'FFFFU, 0xFFFF'FFF8U);
}());

// a value above Max clamps: Fuse_Traits caps the WDT period at 0xB
static_assert(Wdt{row(0xFFFF'FFFFU,
                      0xFFFF'FFFFU)}
                .period.asValue()
              == 0xB);
}   // namespace

int main() {
    std::printf("fuse_traits: ok\n");
    return 0;
}
