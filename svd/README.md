# SVD files of the further parts

`../chip.svd` is the ATSAMD21G17L (the default part, `KVASIR_ATSAMD21_MPU` unset). It is not the
file Microchip ships: its register, field and enum names are the ones `chip_atsam_common`'s
drivers are written against, shared with the SAM C21.

| file | part | how it is made |
|---|---|---|
| `<build>/chip_svd/ATSAMD21G18A.svd` | ATSAMD21G18A | `scripts/make_g18a_svd.py --out`: `../chip.svd` without the peripherals only the L parts have (AC1, TC6, TC7, TCC3), plus USB and I2S from `samd21a_pack_usb_i2s.svd`, plus PTC's interrupt line 26 (an interrupt-only peripheral, `A_ONLY_INTERRUPTS`) |
| `samd21a_pack_usb_i2s.svd` | - | the USB and I2S `<peripheral>` blocks of Microchip's `Microchip.SAMD21_DFP.3.8.270.atpack` (`samd21a/svd/ATSAMD21G18A.svd`, Apache-2.0, from packs.download.microchip.com), byte for byte; a new pack goes in with `scripts/make_g18a_svd.py --refresh-fragment <pack>/samd21a/svd/ATSAMD21G18A.svd` (then update the version here) |

**The G18A file is generated at configure into the build tree, `<build>/chip_svd/`, and never checked in**:
`cmake/chip.cmake` runs the script for the G18A (the SDK's `kvasir_chip_variant()`, `cmake/chip_package.cmake`), and a change of `../chip.svd`, the
script or the fragment re-runs it. Edit `../chip.svd` (or the script for a G18A-only rule), never the generated file.

Microchip's own `ATSAMD21G18A.svd` cannot be used as it is: the shared drivers do not build
against its names (`EIC::CTRL` instead of `CTRLA`, other DMAC trigger and EVSYS enum names, ...).

The pack nests the USB endpoint registers in a `DEVICE_ENDPOINT[%s]` cluster inside the `DEVICE`
cluster, which the SVD converter turns into `USB_DEVICE::Registers<>::DEVICE_ENDPOINT<N>::EPCFG`
(Kvasir_SDK/svd_converter, "cluster nested in cluster"). `src/chip/Usb_Traits.hpp` names the
registers for the USB backend, so another spelling only changes that header.

The NVMCTRL registers still list the RWW EEPROM commands of the L parts, which this part does not
have.
