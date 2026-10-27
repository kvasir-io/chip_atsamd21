#pragma once

#include "ClockLimits.hpp"
#include "WaitBounds.hpp"
#include "atsam_common/GclkDivision.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/GCLK.hpp"
#include "peripherals/PM.hpp"
#include "peripherals/SYSCTRL.hpp"

#include <bit>
#include <cstdint>

namespace Kvasir { namespace GCLK {
    // The generated enums of the SVD (chip.svd, the G18A's from scripts/make_g18a_svd.py): GENCTRL.SRC (DS40001882L
    // 15.8.4) and CLKCTRL.ID (15.8.3, md l.5191-5240) - each part names only the channels it has (G17L: TCC3, TC6_TC7;
    // G18A: USB, PTC, I2S_0/1).
    using GeneratorSource = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL::SRCVal;
    using Peripheral      = Kvasir::Peripheral::GCLK::Registers<>::CLKCTRL::IDVal;

    template<unsigned Generator, GeneratorSource Source, unsigned long long Div>
    struct GenericClockGenerator {
        using GC = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL;
        using GD = Kvasir::Peripheral::GCLK::Registers<>::GENDIV;

        static_assert(
          Sam::Gclk::divisionWritable(Div,
                                      ClockLimits::D21::gclkDivBits(Generator)),
          "GCLK division: GENDIV.DIV does not hold it, and it is no power of 2 (DIVSEL)");
        static constexpr Sam::Gclk::Division division
          = Sam::Gclk::divisionOf(Div, ClockLimits::D21::gclkDivBits(Generator));

        static constexpr bool divisionInTable = Div <= ClockLimits::D21::gclkMaxDivision(Generator);

        [[nodiscard]] static constexpr auto enable() {
            // The data sheet's "Maximum Division Factor" (15.8.5, md l.5407) is what the hardware
            // does: above it the division clamps (/512 for DIVSEL on rev D silicon).
            static_assert(
              divisionInTable,
              "GCLK generator division above the SAM D21's Maximum Division Factor (data "
              "sheet 15.8.5: 512 for generators 0 and 3-8, 64 for 2, 131072 for 1): the "
              "silicon clamps it (measured), so the clock would not be what is written");
            return list(
              Register::sequencePoint,
              write(GD::id, Register::value<Generator>()),
              write(GD::div, Register::value<division.div>()),
              Register::sequencePoint,
              GC::overrideDefaults(
                write(GC::id, Register::value<Generator>()),
                set(GC::genen),
                write(
                  GC::divsel,
                  Register::value<typename GC::DIVSELVal,
                                  division.divsel ? GC::DIVSELVal::div2 : GC::DIVSELVal::div1>()),
                set(GC::runstdby),
                write(GC::src, Register::value<GeneratorSource, Source>())));
        }
    };

    template<unsigned Generator, Peripheral Peripheral>
    struct PeripheralChannelController {
        using CC = Kvasir::Peripheral::GCLK::Registers<>::CLKCTRL;

        [[nodiscard]] static constexpr auto enable() {
            return list(Register::sequencePoint,
                        CC::overrideDefaults(
                          write(CC::id, Register::value<GCLK::Peripheral, Peripheral>()),
                          set(CC::clken),
                          write(CC::gen,
                                Register::value<typename CC::GENVal,
                                                static_cast<typename CC::GENVal>(Generator)>())));
        }
    };

    // GCLK0's source: GENCTRL is indirect - an 8-bit write of the ID, then a read (15.6.4.1); SRC is bits 12:8
    // (15.8.4).
    inline GeneratorSource gclk0Source() {
        using GenCtrl = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL;
        Kvasir::Register::writeLane<std::uint8_t, 0>(GenCtrl::FULLREGISTER,
                                                     std::uint8_t{0});   // GENCTRL.ID = 0
        return get<0>(apply(read(GenCtrl::src)));
    }
}}   // namespace Kvasir::GCLK
