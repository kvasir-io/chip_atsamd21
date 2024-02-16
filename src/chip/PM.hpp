#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#include <cstdint>
#include <string_view>

namespace Kvasir { namespace PM {
    template<unsigned baseAddr = 0x40000400>
    struct Registers {
        static constexpr unsigned BaseAddr = baseAddr;

        struct RCAUSE {
            using Addr = Register::Address<baseAddr + 0x38, 0xff, 0x00, unsigned char>;
            static constexpr Register::FieldLocation<
              Addr,
              Register::maskFromRange(2, 0, 6, 4),
              Register::ReadWriteAccess,
              unsigned char>
              flags{};
            static constexpr Register::FieldValue<typename decltype(flags)::type, 0b0000'0001>
              POR{};
            static constexpr Register::FieldValue<typename decltype(flags)::type, 0b0000'0010>
              BOD12{};
            static constexpr Register::FieldValue<typename decltype(flags)::type, 0b0000'0100>
              BOD33{};
            static constexpr Register::FieldValue<typename decltype(flags)::type, 0b0001'0000>
              EXT{};
            static constexpr Register::FieldValue<typename decltype(flags)::type, 0b0010'0000>
              WDT{};
            static constexpr Register::FieldValue<typename decltype(flags)::type, 0b0100'0000>
              SYST{};
        };
    };

    namespace Detail {
        template<unsigned Offset, int BitPos>
        using BitSet = Register::Action<
          Kvasir::Register::WOBitLocT<Register::Address<Registers<>::BaseAddr + Offset>, BitPos>,
          Kvasir::Register::WriteLiteralAction<(1U << unsigned(BitPos))>>;

        template<unsigned Offset, int BitPos>
        using BitClear = Register::Action<
          Kvasir::Register::WOBitLocT<Register::Address<Registers<>::BaseAddr + Offset>, BitPos>,
          Kvasir::Register::WriteLiteralAction<0>>;

        struct PeripheryEnableInfo {
            unsigned address;
            unsigned offset;
            unsigned bit;
        };

        static constexpr std::array peripheryEnableInfos{
          PeripheryEnableInfo{0x40001800, 0x18,  6}, // EIC
          PeripheryEnableInfo{0x41004800, 0x14,  5}, // DMAC
          PeripheryEnableInfo{0x42000400, 0x20,  1}, // EVSYS
          PeripheryEnableInfo{0x42000800, 0x20,  2}, // SERCOM0
          PeripheryEnableInfo{0x42000C00, 0x20,  3}, // SERCOM1
          PeripheryEnableInfo{0x42001000, 0x20,  4}, // SERCOM2
          PeripheryEnableInfo{0x42001400, 0x20,  5}, // SERCOM3
          PeripheryEnableInfo{0x42001800, 0x20,  6}, // SERCOM4
          PeripheryEnableInfo{0x42001C00, 0x20,  7}, // SERCOM5
          PeripheryEnableInfo{0x42002000, 0x20,  8}, // TCC0
          PeripheryEnableInfo{0x42002400, 0x20,  9}, // TCC1
          PeripheryEnableInfo{0x42002800, 0x20, 10}, // TCC2
          PeripheryEnableInfo{0x42004000, 0x20, 16}, // ADC
          PeripheryEnableInfo{0x42004400, 0x20, 17}, // AC0
          PeripheryEnableInfo{0x42004800, 0x20, 18}, // DAC
          PeripheryEnableInfo{0x42005400, 0x20, 21}, // AC1
        };

        static constexpr bool isValidPeripheryAddress(unsigned peripheryAddress) {
            for(auto pei : peripheryEnableInfos) {
                if(pei.address == peripheryAddress) {
                    return true;
                }
            }
            return false;
        }

        static constexpr unsigned getOffset(unsigned peripheryAddress) {
            for(auto pei : peripheryEnableInfos) {
                if(pei.address == peripheryAddress) {
                    return pei.offset;
                }
            }
            return 0;
        }

        static constexpr unsigned getBit(unsigned peripheryAddress) {
            for(auto pei : peripheryEnableInfos) {
                if(pei.address == peripheryAddress) {
                    return pei.bit;
                }
            }
            return 0;
        }

    }   // namespace Detail
    template<unsigned PeripheryAddress>
    struct enable {
        static_assert(
          Detail::isValidPeripheryAddress(PeripheryAddress),
          "invalid PeripheryAddress to enable");
        using action
          = Detail::BitSet<Detail::getOffset(PeripheryAddress), Detail::getBit(PeripheryAddress)>;
    };

    template<unsigned PeripheryAddress>
    struct disable {
        static_assert(
          Detail::isValidPeripheryAddress(PeripheryAddress),
          "invalid PeripheryAddress to disable");
        using action
          = Detail::BitClear<Detail::getOffset(PeripheryAddress), Detail::getBit(PeripheryAddress)>;
    };

    enum class ResetCause : std::uint8_t { por, bod12, bod33, ext, wdt, syst };

    inline ResetCause reset_cause() {
        auto c = apply(read(Registers<>::RCAUSE::flags));
        if(c == Registers<>::RCAUSE::POR) {
            return ResetCause::por;
        }
        if(c == Registers<>::RCAUSE::BOD12) {
            return ResetCause::bod12;
        }
        if(c == Registers<>::RCAUSE::BOD33) {
            return ResetCause::bod33;
        }
        if(c == Registers<>::RCAUSE::EXT) {
            return ResetCause::ext;
        }
        if(c == Registers<>::RCAUSE::WDT) {
            return ResetCause::wdt;
        }
        return ResetCause::syst;
    }

}}   // namespace Kvasir::PM
