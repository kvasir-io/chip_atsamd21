#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>

namespace Kvasir { namespace PM {
    template<unsigned baseAddr = 0x40000400>
    struct Registers {
        static constexpr unsigned BaseAddr = baseAddr;

        struct RCAUSE {
            using Addr = Register::Address<baseAddr + 0x38, 0xff, 0x00, unsigned char>;
            static constexpr Register::FieldLocation<Addr,
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
#if defined(KVASIR_CHIP_ATSAMD21G18A)
          // Two masks gate the USB: the AHB clock (AHBMASK bit 6) and the APB one (APBBMASK,
          // offset 0x1C, bit 5). SAM D21 datasheet DS40001882, PM register descriptions.
          PeripheryEnableInfo{0x41005000, 0x14,  6}, // USB (AHB)
          PeripheryEnableInfo{0x41005000, 0x1C,  5}, // USB (APB)
          PeripheryEnableInfo{0x42005000, 0x20, 20}, // I2S
#else
          PeripheryEnableInfo{0x42005400, 0x20, 21},   // AC1
#endif
        };

        static constexpr bool isValidPeripheryAddress(unsigned peripheryAddress) {
            for(auto pei : peripheryEnableInfos) {
                if(pei.address == peripheryAddress) { return true; }
            }
            return false;
        }

        // A peripheral has one row per mask bit that gates it: usually one, the USB two.
        static constexpr std::size_t rowCount(unsigned peripheryAddress) {
            std::size_t n{};
            for(auto pei : peripheryEnableInfos) {
                if(pei.address == peripheryAddress) { ++n; }
            }
            return n;
        }

        static constexpr PeripheryEnableInfo row(unsigned    peripheryAddress,
                                                 std::size_t index) {
            for(auto pei : peripheryEnableInfos) {
                if(pei.address == peripheryAddress && index-- == 0) { return pei; }
            }
            return {};
        }

        // One mask bit: the action itself, as it always was. Several: a list of them, which a
        // Startup list takes just the same.
        template<template<unsigned, int> class Bit, unsigned Address, typename Indices>
        struct Actions;

        template<template<unsigned, int> class Bit, unsigned Address>
        struct Actions<Bit, Address, std::index_sequence<0>> {
            using type = Bit<row(Address, 0).offset, int(row(Address, 0).bit)>;
        };

        template<template<unsigned, int> class Bit, unsigned Address, std::size_t... Is>
        struct Actions<Bit, Address, std::index_sequence<Is...>> {
            using type = brigand::list<Bit<row(Address, Is).offset, int(row(Address, Is).bit)>...>;
        };

    }   // namespace Detail

    template<unsigned PeripheryAddress>
    struct enable {
        static_assert(Detail::isValidPeripheryAddress(PeripheryAddress),
                      "invalid PeripheryAddress to enable");
        using action = typename Detail::Actions<
          Detail::BitSet,
          PeripheryAddress,
          std::make_index_sequence<Detail::rowCount(PeripheryAddress)>>::type;
    };

    template<unsigned PeripheryAddress>
    struct disable {
        static_assert(Detail::isValidPeripheryAddress(PeripheryAddress),
                      "invalid PeripheryAddress to disable");
        using action = typename Detail::Actions<
          Detail::BitClear,
          PeripheryAddress,
          std::make_index_sequence<Detail::rowCount(PeripheryAddress)>>::type;
    };

    /// Whether every mask bit that gates the peripheral at `peripheryAddress` is set - what
    /// enable<Address> writes, read back (a self-test's question: "is its bus clock on?").
    inline bool isEnabled(unsigned peripheryAddress) {
        bool all = Detail::isValidPeripheryAddress(peripheryAddress);
        for(auto const pei : Detail::peripheryEnableInfos) {
            if(pei.address != peripheryAddress) { continue; }
            // NOLINTNEXTLINE(performance-no-int-to-ptr)
            auto const* const mask = reinterpret_cast<std::uint32_t const volatile*>(
              std::uintptr_t{Registers<>::BaseAddr} + pei.offset);
            all = all && ((*mask >> pei.bit) & 1U) != 0;
        }
        return all;
    }

    enum class ResetCause : std::uint8_t { por, bod12, bod33, ext, wdt, syst };

    inline ResetCause reset_cause() {
        auto c = apply(read(Registers<>::RCAUSE::flags));
        if(c == Registers<>::RCAUSE::POR) { return ResetCause::por; }
        if(c == Registers<>::RCAUSE::BOD12) { return ResetCause::bod12; }
        if(c == Registers<>::RCAUSE::BOD33) { return ResetCause::bod33; }
        if(c == Registers<>::RCAUSE::EXT) { return ResetCause::ext; }
        if(c == Registers<>::RCAUSE::WDT) { return ResetCause::wdt; }
        return ResetCause::syst;
    }

}}   // namespace Kvasir::PM
