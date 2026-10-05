#pragma once

#include "ClockLimits.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Register/Utility.hpp"
#include "peripherals/SYSCTRL.hpp"

#include <cstdint>

/// The DFLL48M in open loop: the 48 MHz oscillator free-running on DFLLVAL's coarse (the
/// band, 6 bits) and fine (within it, 10 bits) values. Which coarse value gives 48 MHz differs
/// from die to die, so every part carries the one measured for it at the factory in the NVM
/// software calibration area (datasheet DS40001882, 17.6.7.1.1: "Using DFLL48M COARSE CAL ...
/// in DFLL.COARSE helps to output a frequency close to 48 MHz"); enableOpenLoop() uses it
/// unless it is handed a trim. There is no factory value for fine: it is 512, the middle of
/// its range, which is what the electrical characteristics specify the open-loop output
/// for -- 45 / 47 / 49 MHz min / typ / max with COARSE CAL and FINE = 512. A part trimmed by
/// hand against a reference can be closer than that.
namespace Kvasir { namespace DFLL {

    struct Trim {
        std::uint8_t  coarse{};
        std::uint16_t fine{};

        constexpr bool operator==(Trim const&) const = default;
    };

    /// The middle of both ranges: fine always, coarse where the part has no factory value
    /// (the field reads all ones).
    inline constexpr Trim MidRange{.coarse = 0x1F, .fine = 0x200};

    /// The NVM software calibration area, 128 bits from 0x00806020 (datasheet table 10-8):
    /// DFLL48M COARSE CAL is bits 63:58, the top of the second word. Bits 73:64, where older
    /// revisions of the datasheet had a DFLL48M FINE CAL, are reserved and not used.
    inline constexpr std::uint32_t CalibrationAreaAddress = 0x00806020;

    /// The trim out of the calibration area's second word.
    [[nodiscard]] constexpr Trim decode(std::uint32_t word1) {
        auto const coarse = static_cast<std::uint8_t>((word1 >> 26U) & 0x3FU);
        return {.coarse = coarse == 0x3FU ? MidRange.coarse : coarse, .fine = MidRange.fine};
    }

    static_assert(decode(0x94'00'00'00)
                    == Trim{.coarse = 37,
                            .fine   = 0x200},
                  "bits 63:58");
    static_assert(decode(0x97'FF'FF'FF)
                    == Trim{.coarse = 37,
                            .fine   = 0x200},
                  "and nothing of what is below them");
    static_assert(decode(0xFF'FF'FF'FF) == MidRange,
                  "an unprogrammed area is the middle of the range");

    /// What the factory measured for this part.
    [[nodiscard]] inline Trim factoryTrim() {
        auto const* const area = reinterpret_cast<std::uint32_t const volatile*>(
          CalibrationAreaAddress);   // NOLINT(performance-no-int-to-ptr)
        return decode(area[1]);
    }

    /// The DFLL running in open loop on `trim`, ready to be a clock generator's source.
    /// ONDEMAND is cleared with the enable, before DFLLVAL is written: "the DFLL clock must be
    /// requested before being configured; otherwise, a write access to a DFLL register can
    /// freeze the device" (errata DS80000760, 1.2.1, all silicon revisions; its workaround is
    /// this write). The flash wait states for 48 MHz are the caller's, before it switches the
    /// CPU over: `Kvasir::Nvm::waitStates<48'000'000, Supply>()` (ClockTree.hpp) is what the
    /// datasheet allows to name. Table 37-52 gives the open loop up to 49 MHz, which is above f_CPU
    /// max (48 MHz, Table 37-7) and above the last row of Table 37-42: there is no wait-state count
    /// for the open loop's worst case (ClockLimits::D21::DfllOpenLoopMaxHz).
    inline void enableOpenLoop(Trim trim = factoryTrim()) {
        using KSR        = Kvasir::Peripheral::SYSCTRL::Registers<>;
        auto const ready = [] {
            Kvasir::Register::waitUntil<Kvasir::Chip::DfllSyncBound>(
              Kvasir::Register::isSet(KSR::PCLKSR::dfllrdy));
        };

        apply(KSR::DFLLCTRL::overrideDefaults(set(KSR::DFLLCTRL::enable),
                                              clear(KSR::DFLLCTRL::ondemand)));
        ready();
        apply(write(KSR::DFLLVAL::fine, trim.fine),
              write(KSR::DFLLVAL::diff, 0),
              write(KSR::DFLLVAL::coarse, trim.coarse));
        ready();
    }

    /// The DFLL locked to the USB start-of-frame, for a device without a crystal: the host sends
    /// a SOF every millisecond, and with DFLLMUL.MUL = 48000 (0xBB80) the loop makes 48 MHz of
    /// it, inside the +-0.25 % full speed asks for (datasheet 17.6.7.2.2 "USB Clock Recovery
    /// Mode", 32.5.3 "GCLK_USB of 48 MHz +-0.25%"). COARSE is final in this mode - "the value
    /// stored in DFLLVAL.COARSE will be used as final Coarse Value ... loaded from NVM OTP row by
    /// software" - and the loop only searches FINE, from the middle of its range. QLDIS has to
    /// be clear and CCDIS should be set (same section). FSTEP = 10 is the step the electrical
    /// characteristics give the lock time for; CSTEP plays no part with COARSE fixed.
    ///
    /// Until a host is there the DFLL runs on those start values, i.e. as in open loop
    /// (45 / 47 / 49 MHz), which is enough to enumerate: the first SOFs pull it in.
    ///
    /// ONDEMAND is cleared first, errata DS80000760 1.2.1 as in enableOpenLoop(). The lock bits
    /// of PCLKSR are not waited on: in this mode they "have no valid meaning" (17.6.7.2.2) and
    /// can be wrong after a suspend (errata 1.2.3). DFLLRDY is the register synchronisation and
    /// is safe to wait on. The wait states are the caller's, for 48 MHz.
    inline void enableUsbRecovery(Trim trim = factoryTrim()) {
        using KSR        = Kvasir::Peripheral::SYSCTRL::Registers<>;
        auto const ready = [] {
            Kvasir::Register::waitUntil<Kvasir::Chip::DfllSyncBound>(
              Kvasir::Register::isSet(KSR::PCLKSR::dfllrdy));
        };

        apply(KSR::DFLLCTRL::overrideDefaults(set(KSR::DFLLCTRL::enable),
                                              clear(KSR::DFLLCTRL::ondemand)));
        ready();
        apply(write(KSR::DFLLVAL::fine, trim.fine),
              write(KSR::DFLLVAL::diff, 0),
              write(KSR::DFLLVAL::coarse, trim.coarse));
        ready();
        apply(
          KSR::DFLLMUL::overrideDefaults(write(KSR::DFLLMUL::mul, Kvasir::Register::value<48000>()),
                                         write(KSR::DFLLMUL::fstep, Kvasir::Register::value<10>()),
                                         write(KSR::DFLLMUL::cstep, Kvasir::Register::value<1>())));
        ready();
        apply(KSR::DFLLCTRL::overrideDefaults(set(KSR::DFLLCTRL::enable),
                                              set(KSR::DFLLCTRL::mode),
                                              set(KSR::DFLLCTRL::usbcrm),
                                              set(KSR::DFLLCTRL::ccdis),
                                              clear(KSR::DFLLCTRL::qldis),
                                              clear(KSR::DFLLCTRL::ondemand)));
        ready();
    }

    /// The DFLL locked to a reference on GCLK_DFLL48M_REF (generic clock channel 0, which the
    /// caller routes before: `PeripheralChannelController<gen, Peripheral::dfll48>`), the sequence
    /// of 17.6.7.1.2 (DS40001882L, md line 7247): enabled with ONDEMAND off (errata 1.2.1), COARSE
    /// from the factory and BPLCKC set ("will reduce DFLL Lock time to DFLL Fine lock time"),
    /// DFLLMUL, then MODE. f = DFLLMUL.MUL x f_ref.
    ///
    /// Checked at compile time: the reference inside 0.732..33 kHz (Table 37-54, line 42158), MUL
    /// in its 16 bits (17.8.12), MUL x f_ref within TolerancePpm of TargetHz, and the caller's
    /// ReferenceAccuracyPpm within the required 2 % (note 1 under Table 37-54).
    ///
    /// CSTEP/FSTEP 31/511: 50 % of COARSE/FINE (17.6.7.1.2 step 2). DFLLOOB stays off (errata
    /// DS80000760M 1.2.2). Waits for the fine lock (PCLKSR.DFLLLCKF, 17.6.7.1.3), for ever without
    /// a running reference. The wait states for TargetHz are the caller's. Untested on hardware.
    template<std::uint64_t RefHz,
             std::uint64_t ReferenceAccuracyPpm,
             std::uint64_t TargetHz     = 48'000'000,
             std::uint64_t TolerancePpm = 2000>
    inline void enableClosedLoop(Trim trim = factoryTrim()) {
        namespace L = Kvasir::ClockLimits::D21;
        Kvasir::ClockLimits::assertInRange<Kvasir::Prescaler::Rational{RefHz, 1},
                                           L::Dfll48m.refMin,
                                           L::Dfll48m.refMax,
                                           "DFLL48M reference",
                                           "SAM D21 Table 37-54 f_REF">();
        constexpr auto loop = closedLoop<L::Dfll48m>(RefHz, TargetHz);
        static_assert(loop.mulFits, "DFLL48M: DFLLMUL.MUL is 16 bits (17.8.12)");
        Kvasir::Prescaler::assertInTolerance<loop.achieved,
                                             TargetHz,
                                             Kvasir::Prescaler::Tolerance::ppm(TolerancePpm),
                                             "DFLL48M closed loop">();
        static_assert(ReferenceAccuracyPpm <= L::DfllReferenceAccuracyMaxPpm,
                      "DFLL48M: the closed-loop reference must be within 2 % (note 1 under SAM "
                      "D21 Table 37-54)");

        using KSR = Kvasir::Peripheral::SYSCTRL::Registers<>;
        using Kvasir::Register::value;
        auto const ready = [] {
            Kvasir::Register::waitUntil<Kvasir::Chip::DfllSyncBound>(
              Kvasir::Register::isSet(KSR::PCLKSR::dfllrdy));
        };

        apply(KSR::DFLLCTRL::overrideDefaults(set(KSR::DFLLCTRL::enable),
                                              clear(KSR::DFLLCTRL::ondemand)));
        ready();
        apply(write(KSR::DFLLVAL::fine, trim.fine),
              write(KSR::DFLLVAL::diff, 0),
              write(KSR::DFLLVAL::coarse, trim.coarse));
        ready();
        apply(KSR::DFLLMUL::overrideDefaults(
          write(KSR::DFLLMUL::mul, value<loop.mul>()),
          write(KSR::DFLLMUL::fstep, value<MaxFineStep<L::Dfll48m>>()),
          write(KSR::DFLLMUL::cstep, value<MaxCoarseStep<L::Dfll48m>>())));
        ready();
        apply(KSR::DFLLCTRL::overrideDefaults(set(KSR::DFLLCTRL::enable),
                                              set(KSR::DFLLCTRL::mode),
                                              set(KSR::DFLLCTRL::bplckc),
                                              clear(KSR::DFLLCTRL::ondemand)));
        ready();
        Kvasir::Register::waitUntil<Kvasir::Chip::DfllLockBound>(
          Kvasir::Register::isSet(KSR::PCLKSR::dflllckf));
    }
}}   // namespace Kvasir::DFLL
