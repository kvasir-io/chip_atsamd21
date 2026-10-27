#pragma once
// The fields of the NVM User Row (SAM D21 DS40001882L 10.3.1, Table 10-7; at 0x804000). Bits 24:17 and 41 are BOD12
// calibration written in production that "must not be changed": no field here covers them, and combineInto leaves
// them as read. WDT Window resets to 0xB on silicon, not the table's 0x5 (errata DS80000760M 1.5.4, md l.620-623):
// revisions C and D only - the erratum's revision table read on the PDF page 12, its .md lost the columns - so a
// revision E or later part has the table's 0x5 (the default below is a rev C/D part's).
#include "kvasir/Register/BitField.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace Kvasir { namespace Fuses { namespace Traits {
    struct Bod33 {
        enum class Action : std::uint8_t {
            none      = 0,
            reset     = 1,
            interrupt = 2,
        };
        enum class Enable : std::uint8_t {
            disabled = 0,
            enabled  = 1,
        };

        enum class Hysteresis : std::uint8_t {
            disabled = 0,
            enabled  = 1,
        };

        struct Level {
            std::uint8_t level;
        };

        BitField<16, 15, 0, 2, Action>     action;
        BitField<14, 14, 0, 1, Enable>     enable;
        BitField<40, 40, 0, 1, Hysteresis> hysteresis;
        BitField<13, 8>                    level;

        constexpr Bod33(Action     action_,
                        Enable     enable_,
                        Hysteresis hysteresis_,
                        Level      level_)
          : action{action_}
          , enable{enable_}
          , hysteresis{hysteresis_}
          , level{level_.level} {}

        template<std::size_t N>
        constexpr explicit Bod33(std::array<std::byte,
                                            N> const& a)
          : action{a}
          , enable{a}
          , hysteresis{a}
          , level{a} {}
    };

    struct Wdt {
        enum class Enable : std::uint8_t {
            disabled = 0,
            enabled  = 1,
        };

        enum class AlwaysOn : std::uint8_t {
            disabled = 0,
            enabled  = 1,
        };
        enum class WindowEnable : std::uint8_t {
            disabled = 0,
            enabled  = 1,
        };

        struct Period {
            std::uint8_t period;
        };

        struct Window {
            std::uint8_t window;
        };

        struct Offset {
            std::uint8_t offset;
        };

        BitField<25, 25, 0, 1, Enable>       enable;
        BitField<26, 26, 0, 1, AlwaysOn>     alwaysOn;
        BitField<39, 39, 0, 1, WindowEnable> windowEnable;
        BitField<30, 27, 0, 0x0B>            period;
        BitField<34, 31, 0, 0x0B>            window;
        BitField<38, 35, 0, 0x0B>            offset;

        constexpr Wdt(Enable       enable_,
                      AlwaysOn     alwaysOn_,
                      WindowEnable windowEnable_,
                      Period       period_,
                      Window       window_,
                      Offset       offset_)
          : enable{enable_}
          , alwaysOn{alwaysOn_}
          , windowEnable{windowEnable_}
          , period{period_.period}
          , window{window_.window}
          , offset{offset_.offset} {}

        template<std::size_t N>
        constexpr explicit Wdt(std::array<std::byte,
                                          N> const& a)
          : enable{a}
          , alwaysOn{a}
          , windowEnable{a}
          , period{a}
          , window{a}
          , offset{a} {}
    };

    struct Nvm {
        struct Bootprot {
            std::uint8_t bootprot;
        };

        struct Eeprom {
            std::uint8_t eeprom;
        };

        struct Lock {
            std::uint16_t lock;
        };

        BitField<2, 0>   bootprot;
        BitField<6, 4>   eeprom;
        BitField<63, 48> lock;

        constexpr Nvm(Bootprot bootprot_,
                      Eeprom   eeprom_,
                      Lock     lock_)
          : bootprot{bootprot_.bootprot}
          , eeprom{eeprom_.eeprom}
          , lock{lock_.lock} {}

        template<std::size_t N>
        constexpr explicit Nvm(std::array<std::byte,
                                          N> const& a)
          : bootprot{a}
          , eeprom{a}
          , lock{a} {}
    };
}}}   // namespace Kvasir::Fuses::Traits
