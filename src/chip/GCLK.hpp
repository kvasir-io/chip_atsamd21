#pragma once

#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/GCLK.hpp"

namespace Kvasir { namespace GCLK {
    enum class GeneratorSource : unsigned {
        xosc      = 0,   //XOSC oscillator output
        gclkin    = 1,   //Generator input pad
        gclkgen1  = 2,   //Generic clock generator 1 output
        osculp32k = 3,   //OSCULP32K oscillator output
        osc32k    = 4,   //OSC32K oscillator output
        xosc32k   = 5,   //XOSC32K oscillator output
        osc8m     = 6,   //OSC8M oscillator output
        dfll48m   = 7,   //DFLL48M output
        fdpll     = 8,   //FDPLL output
    };

    enum class Peripheral : std::uint16_t {
        dfll48       = 0,    //DFLL48
        fdpll        = 1,    //FDPLL
        fdpll32k     = 2,    //FDPLL32K
        wdt          = 3,    //WDT
        rtc          = 4,    //RTC
        eic          = 5,    //EIC
        evsys_0      = 7,    //EVSYS_0
        evsys_1      = 8,    //EVSYS_1
        evsys_2      = 9,    //EVSYS_2
        evsys_3      = 10,   //EVSYS_3
        evsys_4      = 11,   //EVSYS_4
        evsys_5      = 12,   //EVSYS_5
        evsys_6      = 13,   //EVSYS_6
        evsys_7      = 14,   //EVSYS_7
        evsys_8      = 15,   //EVSYS_8
        evsys_9      = 16,   //EVSYS_9
        evsys_10     = 17,   //EVSYS_10
        evsys_11     = 18,   //EVSYS_11
        sercomx_slow = 19,   //SERCOMX_SLOW
        sercom0_core = 20,   //SERCOM0_CORE
        sercom1_core = 21,   //SERCOM1_CORE
        sercom2_core = 22,   //SERCOM2_CORE
        sercom3_core = 23,   //SERCOM3_CORE
        sercom4_core = 24,   //SERCOM4_CORE
        sercom5_core = 25,   //SERCOM5_CORE
        tcc0_tcc1    = 26,   //TCC0_TCC1
        tcc2_tc3     = 27,   //TCC2_TC3
        tc4_tc5      = 28,   //TC4_TC5
        tc6_tc7      = 29,   //TC6_TC7
        adc          = 30,   //ADC
        ac_dig       = 31,   //AC_DIG
        ac_ana       = 32,   //AC_ANA
        dac          = 33,   //DAC
        i2s_0        = 35,   //I2S_0
        i2s_1        = 36,   //I2S_1
    };

    template<unsigned Generator, GeneratorSource Source, unsigned long long Div>
    struct GenericClockGenerator {
        using GC = Kvasir::Peripheral::GCLK::Registers<>::GENCTRL;
        using GD = Kvasir::Peripheral::GCLK::Registers<>::GENDIV;

        static constexpr unsigned maxDiv
          = (Generator == 1 ? (1 << 16) : (Generator == 2 ? (1 << 5) : (1 << 8))) - 1;

        static_assert(
          (Div <= maxDiv) || (std::popcount(Div) == 1),
          "must be power of 2 or lower then maxDiv");

        static constexpr auto divsel = []() {
            if constexpr(Div > maxDiv) {
                return GC::DIVSELValC::div2;
            } else {
                return GC::DIVSELValC::div1;
            }
        }();
        static constexpr auto div = []() {
            if constexpr(Div > maxDiv) {
                return std::countr_zero(Div) - 1;
            } else {
                return Div;
            }
        }();

        [[nodiscard]] static constexpr auto enable() {
            return list(
              Register::sequencePoint,
              write(GD::id, Register::value<Generator>()),
              write(GD::div, Register::value<div>()),
              Register::sequencePoint,
              Register::overrideDefaults<typename GC::default_values>::value(
                write(GC::id, Register::value<Generator>()),
                set(GC::genen),
                write(divsel),
                set(GC::runstdby),
                write(
                  GC::src,
                  Register::
                    value<typename GC::SRCVal, static_cast<typename GC::SRCVal>(Source)>())));
        }
    };

    template<unsigned Generator, Peripheral Peripheral>
    struct PeripheralChannelController {
        using CC = Kvasir::Peripheral::GCLK::Registers<>::CLKCTRL;
        [[nodiscard]] static constexpr auto enable() {
            return list(
              Register::sequencePoint,
              Register::overrideDefaults<typename CC::default_values>::value(
                write(
                  CC::id,
                  Register::
                    value<typename CC::IDVal, static_cast<typename CC::IDVal>(Peripheral)>()),
                set(CC::clken),
                write(
                  CC::gen,
                  Register::
                    value<typename CC::GENVal, static_cast<typename CC::GENVal>(Generator)>())));
        }
        [[nodiscard]] static constexpr auto disable() {
            return list(
              Register::sequencePoint,
              Register::overrideDefaults<typename CC::default_values>::value(
                write(
                  CC::id,
                  Register::
                    value<typename CC::IDVal, static_cast<typename CC::IDVal>(Peripheral)>()),
                clear(CC::clken),
                write(
                  CC::gen,
                  Register::
                    value<typename CC::GENVal, static_cast<typename CC::GENVal>(Generator)>())));
        }
    };

}}   // namespace Kvasir::GCLK
