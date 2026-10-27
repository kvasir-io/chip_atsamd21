#pragma once

#include "chip/Fuse_Traits.hpp"
#include "chip/atsam_common/NVMCTRL.hpp"
#include "kvasir/Register/BitField.hpp"
#include "kvasir/Register/Memory.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "uc_log/uc_log.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <utility>

namespace Kvasir { namespace Fuses {

    namespace detail {
        // A group of Fuse_Traits fields (Traits::Bod33, Wdt, Nvm) with isValid / == / combineInto over the fields
        // named by their member pointers, in the order given.
        template<typename Group, auto... Fields>
        struct FieldGroup : Group {
            using Group::Group;

            template<std::size_t N>
            static constexpr bool isValid(std::array<std::byte,
                                                     N> const& a) {
                return (
                  std::remove_cvref_t<decltype(std::declval<Group const&>().*Fields)>::isValid(a)
                  && ...);
            }

            constexpr bool operator==(FieldGroup const& rhs) const {
                return ((group().*Fields == rhs.group().*Fields) && ...);
            }

            template<std::size_t N>
            constexpr void combineInto(std::array<std::byte,
                                                  N>& a) const {
                ((group().*Fields).combineInto(a), ...);
            }

        private:
            [[nodiscard]] constexpr Group const& group() const { return *this; }
        };
    }   // namespace detail

    struct Bod33
      : detail::FieldGroup<Traits::Bod33,
                           &Traits::Bod33::action,
                           &Traits::Bod33::enable,
                           &Traits::Bod33::hysteresis,
                           &Traits::Bod33::level> {
        using FieldGroup::FieldGroup;
    };

    struct Wdt
      : detail::FieldGroup<Traits::Wdt,
                           &Traits::Wdt::enable,
                           &Traits::Wdt::alwaysOn,
                           &Traits::Wdt::windowEnable,
                           &Traits::Wdt::period,
                           &Traits::Wdt::window,
                           &Traits::Wdt::offset> {
        using FieldGroup::FieldGroup;
    };

    struct Nvm
      : detail::FieldGroup<Traits::Nvm,
                           &Traits::Nvm::bootprot,
                           &Traits::Nvm::eeprom,
                           &Traits::Nvm::lock> {
        using FieldGroup::FieldGroup;
    };

    struct FuseBits {
        Bod33 bod33;
        Wdt   wdt;
        Nvm   nvm;

        constexpr FuseBits(Bod33 bod33_,
                           Wdt   wdt_,
                           Nvm   nvm_)
          : bod33(bod33_)
          , wdt(wdt_)
          , nvm{nvm_} {}

        template<std::size_t N>
        constexpr explicit FuseBits(std::array<std::byte,
                                               N> const& a)
          : bod33{a}
          , wdt{a}
          , nvm{a} {}

        template<std::size_t N>
        static constexpr bool isValid(std::array<std::byte,
                                                 N> const& a) {
            return decltype(bod33)::isValid(a) && decltype(wdt)::isValid(a)
                && decltype(nvm)::isValid(a);
        }

        constexpr bool operator==(FuseBits const& rhs) const {
            return bod33 == rhs.bod33 && wdt == rhs.wdt && nvm == rhs.nvm;
        }

        constexpr bool operator!=(FuseBits const& rhs) const { return !(*this == rhs); }

        template<std::size_t N>
        constexpr void combineInto(std::array<std::byte,
                                              N>& a) const {
            bod33.combineInto(a);
            wdt.combineInto(a);
            nvm.combineInto(a);
        }
    };

    enum class FuseWrite : std::uint8_t {
        upToDate,         ///< the user row already held them
        written,          ///< written; they take effect after the next reset (DS40001882L 22.6.3)
        securityBitSet,   ///< not written: the security bit is set (only a chip erase clears it)
    };

    /// Writes the user row's fuse fields where they differ from `soll` (the other bits kept) and, with `lock`, sets
    /// the security bit. Every NVM command is checked (Nvm::command in NVMCTRL.hpp: EAR, PBC, WAP, SSB, each after READY),
    /// and the two words are read back after WAP (Error::verify). Callers run it at every boot, so a failed
    /// write is tried again at the next one.
    [[nodiscard]] inline ::Kvasir::Nvm::Result<FuseWrite> setFuseBits(FuseBits const& soll,
                                                                      bool lock = false) {
        using NVME   = Kvasir::Peripheral::NVMCTRL::Registers<>;
        using Erase  = NVMCTRL::Traits::EraseBound;
        using Write  = NVMCTRL::Traits::WriteBound;
        using Result = ::Kvasir::Nvm::Result<FuseWrite>;

        // the NVM User Row (DS40001882L 10.3.1, Table 10-7): two words, memory, not a register
        static constexpr std::uint32_t fuseBitAddr = 0x00804000;
        auto const                     readRow     = [] {
            std::array<std::uint32_t, 2> bits{};
            Kvasir::Memory::loadWords(fuseBitAddr, bits.data(), bits.size());
            return bits;
        };
        std::array<std::byte, 8> const oldFuseBits = [&readRow]() {
            std::array<std::uint32_t, 2> const bits = readRow();
            std::array<std::byte, 8>           buff{};
            std::memcpy(buff.data(), bits.data(), buff.size());
            return buff;
        }();

        if(FuseBits::isValid(oldFuseBits) && soll == FuseBits{oldFuseBits}) {
            UC_LOG_D("fuse bits already up to date");
            return FuseWrite::upToDate;
        }
        if(apply(read(NVME::STATUS::sb))) {
            UC_LOG_E("want to write fuse bits but securityBit set");
            return FuseWrite::securityBitSet;
        }

        std::array<std::byte, 8> newFuseBits = oldFuseBits;
        soll.combineInto(newFuseBits);
        std::array<std::uint32_t, 2> bits{};
        std::memcpy(bits.data(), newFuseBits.data(), newFuseBits.size());

        ::Kvasir::Nvm::CacheOff const cacheOff{};
        auto const                    cmd
          = [](auto value) { apply(write(value), write(NVME::CTRLA::CMDEXValC::key)); };
        if(auto const r = ::Kvasir::Nvm::command<Erase>([&] {
               NVMCTRL::Traits::setAddr(fuseBitAddr);
               cmd(NVME::CTRLA::CMDValC::ear);
           });
           !r)
        {
            return Result{std::unexpect, r.error()};
        }
        if(auto const r = ::Kvasir::Nvm::command<Write>([&] { cmd(NVME::CTRLA::CMDValC::pbc); });
           !r)
        {
            return Result{std::unexpect, r.error()};
        }
        Kvasir::Memory::store<std::uint32_t>(fuseBitAddr, bits[0]);
        Kvasir::Memory::store<std::uint32_t>(fuseBitAddr + 4, bits[1]);
        if(auto const r = ::Kvasir::Nvm::command<Write>([&] {
               NVMCTRL::Traits::setAddr(fuseBitAddr);
               cmd(NVME::CTRLA::CMDValC::wap);
           });
           !r)
        {
            return Result{std::unexpect, r.error()};
        }
        if(readRow() != bits) { return Result{std::unexpect, ::Kvasir::Nvm::Error::verify}; }

        // "After issuing the SSB command, the PROGE error bit can be checked" (22.6.6)
        if(lock) {
            if(auto const r
               = ::Kvasir::Nvm::command<Write>([&] { cmd(NVME::CTRLA::CMDValC::ssb); });
               !r)
            {
                return Result{std::unexpect, r.error()};
            }
        }

        UC_LOG_D("fuse bits written, they take effect after the next reset");
        return FuseWrite::written;
    }

}}   // namespace Kvasir::Fuses
