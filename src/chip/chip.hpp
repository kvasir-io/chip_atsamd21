#pragma once

#include "Variant.hpp"
//
#include "peripherals/AC.hpp"
#include "peripherals/ADC.hpp"
#include "peripherals/DAC.hpp"
#include "peripherals/DMAC.hpp"
#include "peripherals/DSU.hpp"
#include "peripherals/EIC.hpp"
#include "peripherals/EVSYS.hpp"
#include "peripherals/GCLK.hpp"
#include "peripherals/HMATRIX.hpp"
#include "peripherals/MTB.hpp"
#include "peripherals/NVMCTRL.hpp"
#include "peripherals/PAC.hpp"
#include "peripherals/PM.hpp"
#include "peripherals/PORT.hpp"
#include "peripherals/RTC_MODE0.hpp"
#include "peripherals/RTC_MODE1.hpp"
#include "peripherals/RTC_MODE2.hpp"
#include "peripherals/SERCOM_I2CM.hpp"
#include "peripherals/SERCOM_I2CS.hpp"
#include "peripherals/SERCOM_SPI.hpp"
#include "peripherals/SERCOM_USART.hpp"
#include "peripherals/SYSCTRL.hpp"
#include "peripherals/TCC.hpp"
#include "peripherals/TC_COUNT16.hpp"
#include "peripherals/TC_COUNT32.hpp"
#include "peripherals/TC_COUNT8.hpp"
#include "peripherals/WDT.hpp"
#if defined(KVASIR_CHIP_ATSAMD21G18A)
    #include "peripherals/I2S.hpp"
    #include "peripherals/USB_DEVICE.hpp"
#endif
//
#include "PM.hpp"
//
#include "DFLL.hpp"
#include "GCLK.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "atsam_common/EIC.hpp"
#include "atsam_common/EVSYS.hpp"
#include "atsam_common/Fuses.hpp"

//#include "PM.hpp"

//#include "TC.hpp"
//#include "TCC.hpp"
//#include "WDT.hpp"
//
#include "Sercom_Traits.hpp"
//
#if !defined(KVASIR_CHIP_ATSAMD21G18A)
    // EEPROM emulation in the RWW flash section, which only the L (and D) parts have.
    #include "atsam_common/NVMCTRL.hpp"
#endif
#include "atsam_common/SamPushButton.hpp"
#include "atsam_common/SamRotaryEncoder.hpp"
#include "atsam_common/Sercom_I2C.hpp"
#include "atsam_common/Sercom_I2CQueued.hpp"
#include "atsam_common/Sercom_SPI.hpp"
#include "atsam_common/Sercom_Usart.hpp"
#include "atsam_common/Serial_Number.hpp"
#include "atsam_common/StartUp.hpp"
#include "core/core.hpp"
#if defined(KVASIR_CHIP_ATSAMD21G18A) && __has_include(<kvasir/Devices/USB/Device.hpp>)
    // The USB controller as a backend of kvasir_devices' USB device: only for a firmware that has
    // kvasir_devices on its include path (LIBRARIES kvasir::devices).
    #include "Usb_Traits.hpp"
    #include "atsam_common/usb/Backend.hpp"
#endif
