// The chip contract of chip_atsamd21 as it is today (Kvasir_SDK tests/conformance/ChipContract.hpp): every row the chip
// has is REQUIRE, every row it lacks is GAP - a GAP that the chip fills fails here and moves to REQUIRE.

// the config header first: the chip below needs its log backend and log clock (uc_log.hpp would read it too, but the
// chip headers before it need the same)
#include "kvasir_config.hpp"

// then the chip and the drivers
#include "chip/WaitBounds.hpp"
#include "chip/atsam_common/ImageCheck.hpp"
#include "chip/atsam_common/NVMCTRL.hpp"
#include "chip/atsam_common/Serial_Number.hpp"
#include "chip/chip.hpp"
// the core's debug query and feature macros
#include "core/Features.hpp"
#include "cortex_common/Debug.hpp"

// last: the contract rows see only what is declared before them
#include "ChipContract.hpp"

namespace Conformance {
// the last two rows of the smallest part's main array (ATSAMD21G17L: 128 KB, 256-byte rows)
using Region = Kvasir::Nvm::Region<0x1'FE00, 2>;

// the watchdog, gated (the health supervisor's) and ungated (a Startup entry)
struct WatchdogConfig {
    static constexpr auto clockSpeed  = 1'024;   // GCLK_WDT
    static constexpr auto overrunTime = std::chrono::seconds{2};
    static constexpr bool gatedFeed   = true;
};

struct UngatedWatchdogConfig {
    static constexpr auto clockSpeed  = 1'024;   // GCLK_WDT
    static constexpr auto overrunTime = std::chrono::seconds{1};
};
}   // namespace Conformance

// Startup hooks the SDK's Startup calls
KVASIR_CONFORMANCE_REQUIRE(
  "Startup::FirstInitStep<Tag::User>",
  requires { Kvasir::Startup::FirstInitStep<Kvasir::Tag::User>{}(); });

// the interrupt table and the pins
KVASIR_CONFORMANCE_REQUIRE("Nvic::InterruptOffsetTraits<Tag::User>::end",
                           Kvasir::Nvic::InterruptOffsetTraits<Kvasir::Tag::User>::end > 0);
KVASIR_CONFORMANCE_REQUIRE("Io::PinLocationTraits<void>",
                           Kvasir::Io::PinLocationTraits<void>::pinEnd > 0);

// reset cause, wait bounds
KVASIR_CONFORMANCE_REQUIRE("PM::resetCause()",
                           Kvasir::Conformance::Row::resetCause);
KVASIR_CONFORMANCE_REQUIRE("PM::resetKind()",
                           Kvasir::Conformance::Row::resetKind);
KVASIR_CONFORMANCE_REQUIRE("Chip::part / partName, the identity constants",
                           Kvasir::Conformance::Row::chipPart);
KVASIR_CONFORMANCE_REQUIRE("Register::CpuHzCeiling<Tag::User>",
                           Kvasir::Conformance::Row::cpuHzCeiling);
// every name the shared drivers use (atsam_common/ChipTraits.hpp; StartUp.hpp asserts the same)
KVASIR_CONFORMANCE_REQUIRE("Sam::ChipPackage<Sam::Package>",
                           Kvasir::Sam::ChipPackage<Kvasir::Sam::Package>);
// the CellStore's flash, the image check's backend
KVASIR_CONFORMANCE_REQUIRE("Hal::NorRegion (Nvm::Region)",
                           Kvasir::Hal::NorRegion<Conformance::Region>);
KVASIR_CONFORMANCE_REQUIRE("ImageCheck::Backend (ImageCheck::SamDsu)",
                           Kvasir::ImageCheck::Backend<Kvasir::ImageCheck::SamDsu<>>);
// known gaps (each moves to REQUIRE when the chip gets it)
// a reboot every chip spells the same, "was a debugger seen" (DSU STATUSB.DBGPRES)
KVASIR_CONFORMANCE_REQUIRE("Chip::reboot()",
                           Kvasir::Conformance::Row::chipReboot);
KVASIR_CONFORMANCE_REQUIRE(
  "Port::Reboot<Tag::User>, the reboot hook",
  requires { Kvasir::Port::Reboot<Kvasir::Tag::User>{}(); });
KVASIR_CONFORMANCE_REQUIRE("Chip::debuggerPresent()",
                           Kvasir::Conformance::Row::chipDebuggerPresent);
// the health supervisor's watchdog (atsam_common/Wdt.hpp)
KVASIR_CONFORMANCE_REQUIRE("Kvasir::Watchdog",
                           Kvasir::Conformance::Row::kvasirWatchdog);
// the serial number, as bytes and as hex text (every chip's two functions)
KVASIR_CONFORMANCE_REQUIRE("Kvasir::serialNumber() / serialNumberString()",
                           Kvasir::Conformance::Row::serialNumber);
KVASIR_CONFORMANCE_REQUIRE(
  "health watchdog (gated feed/arm/disarm, Timeout)",
  Kvasir::Conformance::HealthWatchdog<Kvasir::Watchdog<Conformance::WatchdogConfig>>);
KVASIR_CONFORMANCE_REQUIRE(
  "health watchdog, ungated (feed, Timeout)",
  Kvasir::Health::WatchdogShape<Kvasir::Watchdog<Conformance::UngatedWatchdogConfig>>);
// the M0+ cannot read DHCSR itself (its cortex_common/Debug.hpp has haltingDebug() only, `unknown`)
KVASIR_CORE_CONFORMANCE_GAP("Core::Debug::isDebuggerConnected()",
                            Kvasir::Conformance::Row::coreDebugIsDebuggerConnected);
// "is halting debug on": off / on / unknown on every core (cortex_common/Debug.hpp)
KVASIR_CORE_CONFORMANCE_REQUIRE("Core::haltingDebug()",
                                Kvasir::Conformance::Row::coreHaltingDebug);

// The clock gates (chip/Blocks.hpp over atsam_common/ClockGate.hpp). Two masks gate the D21's DMAC (AHBMASK bit 5,
// APBBMASK bit 4) and USB (AHBMASK bit 6, APBBMASK bit 5; DS40001882L 16.8.7, 16.8.9), so enable<> is a list of both;
// both are set out of reset (0x7F), so a write claims nothing. APBCMASK resets to 0x00010000 (16.8.10): a SERCOM
// write claims its gate, an ADC write not, a PAC2 write too (CLK_PAC2_APB off out of reset).
namespace Conformance { namespace Gates {
    namespace P = Kvasir::Peripheral;
    using M     = P::PM::Registers<>;
    using Table = Kvasir::Chip::Blocks::Table;
    static_assert(std::is_same_v<Kvasir::PM::Enable<P::SERCOM_I2CM::Registers<0>::baseAddr>::action,
                                 decltype(set(M::APBCMASK::sercom0_))>);
    static_assert(Table::claimsAt(P::SERCOM_I2CM::Registers<0>::baseAddr) == 1
                  && Table::claimsAt(P::ADC::Registers<>::baseAddr) == 0
                  && Table::claimsAt(P::PAC2::Registers<>::baseAddr) == 1);
    static_assert(std::is_same_v<Kvasir::PM::Enable<P::DMAC::Registers<>::baseAddr>::action,
                                 brigand::list<decltype(set(M::AHBMASK::dmac_)),
                                               decltype(set(M::APBBMASK::dmac_))>>);
    static_assert(Table::claimsAt(P::DMAC::Registers<>::baseAddr) == 0
                  && Table::rowsOf(P::DSU::Registers<>::baseAddr) == 2
                  && Table::rowsOf(P::NVMCTRL::Registers<>::baseAddr) == 2);
#ifdef KVASIR_CHIP_ATSAMD21G18A
    static_assert(std::is_same_v<Kvasir::PM::Enable<P::USB_DEVICE::Registers<>::baseAddr>::action,
                                 brigand::list<decltype(set(M::AHBMASK::usb_)),
                                               decltype(set(M::APBBMASK::usb_))>>);
    static_assert(Table::claimsAt(P::USB_DEVICE::Registers<>::baseAddr) == 0);
#endif
}}   // namespace Conformance::Gates

// GCLK::Peripheral and GeneratorSource are the SVD's generated CLKCTRL.ID / GENCTRL.SRC enums; each part names the
// channels it has (DS40001882L 15.8.3, ID table md l.5191-5240): the G17L TC6_TC7 0x1D and TCC3 0x25, the G18A USB
// 0x06, PTC 0x22, I2S_0/1 0x23/0x24 (scripts/make_g18a_svd.py).
namespace Conformance::GclkIds {
using Kvasir::GCLK::Peripheral;
static_assert(std::is_same_v<Peripheral,
                             Kvasir::Peripheral::GCLK::Registers<>::CLKCTRL::IDVal>
              && std::is_same_v<Kvasir::GCLK::GeneratorSource,
                                Kvasir::Peripheral::GCLK::Registers<>::GENCTRL::SRCVal>);
static_assert(static_cast<unsigned>(Peripheral::dac) == 0x21
              && static_cast<unsigned>(Peripheral::eic) == 0x05
              && static_cast<unsigned>(Kvasir::GCLK::GeneratorSource::fdpll) == 8);
#ifdef KVASIR_CHIP_ATSAMD21G18A
static_assert(static_cast<unsigned>(Peripheral::usb) == 0x06
              && static_cast<unsigned>(Peripheral::ptc) == 0x22
              && static_cast<unsigned>(Peripheral::i2s_0) == 0x23
              && static_cast<unsigned>(Peripheral::i2s_1) == 0x24);
#else
static_assert(static_cast<unsigned>(Peripheral::tc6_tc7) == 0x1D
              && static_cast<unsigned>(Peripheral::tcc3) == 0x25);
#endif
}   // namespace Conformance::GclkIds

// the clock tree's kinds (chip/ClockTree.hpp): the numbers each one takes, no image needed
namespace Conformance::ClockTree {
namespace CS = Kvasir::Clocks::Sam;
// XOSC 8 MHz -> FDPLL96M: DIV 3 (1 MHz), LDR 47, generator 0 / 1, 1 wait state at 2.7 V (Table 37-42)
using Fdpll = CS::Tree<{.cpu = 48'000'000, .source = {.hz = 8'000'000}}>;
static_assert(Fdpll::dpll.div == 3 && Fdpll::dpll.ldr == 47 && Fdpll::dpll.gclkDiv == 1
              && Fdpll::waitStates == 1);
// DFLL48M open loop: the 48 MHz row
using Open = CS::Tree<{.cpu = 48'000'000, .kind = CS::Source::dfllOpenLoop}>;
static_assert(Open::waitStates == 1);
// closed loop on OSC8M (/8 at reset) / 32 = 31.25 kHz: MUL 1536
using Closed = CS::Tree<
  {.cpu = 48'000'000, .kind = CS::Source::dfllClosedLoop, .reference = {.accuracyPpm = 19'500}}>;
static_assert(Closed::refHz == 31'250
              && Kvasir::DFLL::closedLoop<Kvasir::ClockLimits::D21::dfll48m>(Closed::refHz,
                                                                             48'000'000)
                     .mul
                   == 1536);
// 24 MHz from the DFLL: generator 0 / 2, but the open loop's 49 MHz max / 2 = 24.5 MHz is past the
// 0-wait-state row (Table 37-52, Table 37-42): 1 wait state
using Half = CS::Tree<{.cpu = 24'000'000, .kind = CS::Source::dfllOpenLoop}>;
static_assert(Half::waitStates == 1);
// 12 MHz: 49 MHz / 4 = 12.25 MHz, inside the 0-wait-state row
using Quarter = CS::Tree<{.cpu = 12'000'000, .kind = CS::Source::dfllOpenLoop}>;
static_assert(Quarter::waitStates == 0);
// the crystal with the DFLL48M fallback: the FDPLL's numbers, the wait states of the worse branch (48 MHz row); 24 MHz
// takes the divided open loop's 24.5 MHz -> 1 as well, while plain xoscFdpll at 24 MHz needs 0
using Fallback = CS::Tree<
  {.cpu = 48'000'000, .kind = CS::Source::xoscFdpllElseDfll, .source = {.hz = 8'000'000}}>;
static_assert(Fallback::dpll.div == 3 && Fallback::dpll.ldr == 47 && Fallback::waitStates == 1);
using FallbackHalf = CS::Tree<
  {.cpu = 24'000'000, .kind = CS::Source::xoscFdpllElseDfll, .source = {.hz = 8'000'000}}>;
static_assert(FallbackHalf::waitStates == 1
              && CS::Tree<{.cpu    = 24'000'000,
                           .source = {.hz = 8'000'000}}>::waitStates
                   == 0);
#ifdef KVASIR_CHIP_ATSAMD21G18A
// USB recovery without GCLK_USB (a main without the USB device): no ClkUsb provided
using UsbOff
  = CS::Tree<{.cpu = 48'000'000, .kind = CS::Source::dfllUsbRecovery, .usbClock = false}>;
static_assert(std::is_same_v<UsbOff::Provides,
                             brigand::list<Kvasir::Hal::Clk<Kvasir::Clocks::ClkMain,
                                                            48'000'000>,
                                           Kvasir::Startup::ProcessorClock<48'000'000>>>);
#endif
}   // namespace Conformance::ClockTree
