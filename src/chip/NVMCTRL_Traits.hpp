#pragma once

#include "ClockLimits.hpp"
#include "atsam_common/NvmResult.hpp"
#include "kvasir/Register/Wait.hpp"
#include "peripherals/NVMCTRL.hpp"

namespace Kvasir { namespace NVMCTRL { namespace Traits {

    using NVM = Kvasir::Peripheral::NVMCTRL::Registers<>;

    static inline bool ready() { return 0 != apply(read(NVM::INTFLAG::ready)); }

    static inline void command_write_page() {
        apply(write(NVM::CTRLA::CMDValC::wp), write(NVM::CTRLA::CMDEXValC::key));
    }

    static inline void command_erase_row() {
        apply(write(NVM::CTRLA::CMDValC::er), write(NVM::CTRLA::CMDEXValC::key));
    }

    static inline void command_page_buffer_clear() {
        apply(write(NVM::CTRLA::CMDValC::pbc), write(NVM::CTRLA::CMDEXValC::key));
    }

    static inline void command_write_rww() {
        apply(write(NVM::CTRLA::CMDValC::rwweewp), write(NVM::CTRLA::CMDEXValC::key));
    }

    static inline void command_erase_rww() {
        apply(write(NVM::CTRLA::CMDValC::rwweeer), write(NVM::CTRLA::CMDEXValC::key));
    }

    // D21 errata 1.11.3 (DS80000760M, rev E): READY is not cleared by RWWEEER, and a second RWWEEER aborts the
    // first. Workaround: a dummy write to the page buffer right before the command. Done on every revision, no
    // revision check: it only loads one word of the buffer (the next write starts with PBC) and sets ADDR to this
    // row (22.8.8: "automatically updated when writing to the page buffer"), the row the erase is for anyway.
    template<typename T>
    static inline void prepare_erase_rww(T addr) {
        *reinterpret_cast<std::uint32_t volatile*>(addr) = 0xFFFF'FFFFU;
    }

    // D21 errata 1.11.4 (DS80000760M): an NVM command on the RWWEE array with the NVM cache on "can result in
    // incorrect data fetch afterward"; the cache is off around every command (CTRLB.CACHEDIS, 22.8.2; a field write:
    // CTRLB keeps MANW and RWS).
    static inline void enable_cache() { apply(clear(NVM::CTRLB::cachedis)); }

    static inline void disable_cache() { apply(set(NVM::CTRLB::cachedis)); }

    // STATUS.PROGE (bit 2), LOCKE (3), NVME (4) and INTFLAG.ERROR (1) are cleared by writing a one (DS40001882L
    // 22.8.7, 22.8.6); LOAD (STATUS bit 1) is left alone. Only what was read is written back, so a flag that comes up
    // in between stays for the next look. Neither register is PAC-protected (22.5.5).
    static inline std::uint8_t takeErrors() {
        static constexpr std::uint16_t Errors
          = NVM::STATUS::proge.Mask | NVM::STATUS::locke.Mask | NVM::STATUS::nvme.Mask;
        auto const status
          = static_cast<std::uint16_t>(get<0>(apply(read(NVM::STATUS::FULLREGISTER))) & Errors);
        if(status != 0) {
            apply(write(NVM::STATUS::FULLREGISTER, status));
            apply(set(NVM::INTFLAG::error));
        }
        return Kvasir::Nvm::Detail::bits(Kvasir::Register::extract(NVM::STATUS::locke, status) != 0,
                                         Kvasir::Register::extract(NVM::STATUS::proge, status) != 0,
                                         Kvasir::Register::extract(NVM::STATUS::nvme, status) != 0);
    }

    // tFRE (row erase) max 6 ms, tFPP (page programming) max 2.5 ms (DS40001882L Tables 37-45/37-46, md
    // l.41964-41975); x2, at the highest CPU clock (a slower one only lengthens the real time)
    using EraseBound = Kvasir::Register::Bound::Microseconds<12'000, ClockLimits::D21::CpuMax>;
    using WriteBound = Kvasir::Register::Bound::Microseconds<5'000, ClockLimits::D21::CpuMax>;

    template<typename T>
    void set_addr(T addr) {
        apply(write(NVM::ADDR::addr,
                    (reinterpret_cast<std::uint32_t>(addr) / 2)
                      & Kvasir::Register::maskFromRange(21, 0)));
    }

    // Is [addr, addr + bytes) in the main array (mainArray) or the RWWEE array? PARAM (DS40001882L 22.8.3): NVMP 15:0
    // pages, PSZ 18:16 (8 << PSZ bytes a page), RWWEEP 31:20 pages of the RWWEE array at 0x00400000 (Table 10-1).
    // The controller checks nothing: an Erase Row past the end of a 128 KB part wraps around and erases a row of the
    // main array (0x30000 -> 0x10000) with no flag set. The driver asks this before every command.
    static inline bool inArray(std::uint32_t addr,
                               std::uint32_t bytes,
                               bool          mainArray) {
        auto const param = apply(read(NVM::PARAM::nvmp, NVM::PARAM::psz, NVM::PARAM::rwweep));
        std::uint32_t const page = 8U << static_cast<std::uint32_t>(get<1>(param));
        std::uint32_t const base = mainArray ? 0U : 0x0040'0000U;
        std::uint32_t const size = page * (mainArray ? get<0>(param) : get<2>(param));
        return addr >= base && addr - base <= size && bytes <= size - (addr - base);
    }

    static constexpr std::size_t PageSize    = 64;
    static constexpr std::size_t PagesPerRow = 4;
}}}   // namespace Kvasir::NVMCTRL::Traits
