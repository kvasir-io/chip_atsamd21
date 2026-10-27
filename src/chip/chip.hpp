#pragma once
// The chip itself: variant, clocks, power manager, interrupts, Io, start-up, reset cause, reboot and the watchdog.
// No driver and nothing from kvasir_devices: include a driver by name (chip/atsam_common/Sercom_I2CQueued.hpp, ...)
// or all of them through chip/drivers.hpp, and a generated peripheral by name (peripherals/TCC.hpp) or all of them
// through chip/peripherals.hpp. Every header includes the peripherals it uses.
// Any number of an image's TUs may include it, and every other chip header: the Startup itself (vector table, reset
// entry, the runtime-library stand-ins) is kvasir/StartUp/StartUp.hpp, which the main TU alone includes.
#include "Variant.hpp"
//
#include "peripherals/DSU.hpp"
#include "peripherals/GCLK.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "peripherals/PAC0.hpp"
#include "peripherals/PAC1.hpp"
#include "peripherals/PAC2.hpp"
#include "peripherals/PM.hpp"
#include "peripherals/PORT.hpp"
#include "peripherals/SYSCTRL.hpp"
#include "peripherals/WDT.hpp"
//
#include "PM.hpp"
//
#include "ClockTree.hpp"
#include "DFLL.hpp"
#include "Dpll.hpp"
#include "GCLK.hpp"
#include "Interrupt.hpp"
#include "NvmCalibration_Traits.hpp"
#include "Sercom_Traits.hpp"
#include "atsam_common/DMAC.hpp"   // DmaBase with the core: the SERCOM drivers and DMA users take it from here
#include "atsam_common/Debug.hpp"
#include "atsam_common/Io.hpp"
#include "atsam_common/Reboot.hpp"
#include "atsam_common/StartUp.hpp"   // after GCLK.hpp: it calls the GCLK functions non-dependently
#include "atsam_common/Wdt.hpp"
#include "core/core.hpp"
