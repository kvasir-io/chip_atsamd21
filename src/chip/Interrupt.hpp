#pragma once
#include "Variant.hpp"
#include "core/CoreInterrupts.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Port/Interrupts.hpp"
#include "peripherals/Interrupts.hpp"

#include <array>
#include <type_traits>

namespace Kvasir {
// The core's exceptions (core/CoreInterrupts.hpp) and the device lines of this part's SVD (svd_converter:
// peripherals/Interrupts.hpp, the SVD's names lower-cased without IRQ).
// Which lines a part has follows from its SVD: chip.svd for the G17L (TCC3 on 29, DS40001882L Table 11-3), the
// generated G18A SVD for the G18A (USB on 7, I2S on 27, PTC on 26: scripts/make_g18a_svd.py).
struct Interrupt
  : CoreInterrupts
  , Peripheral::DeviceInterrupts {
    // The vector that serves EXTINT line n: one for all sixteen ("EIC - External Interrupt
    // Controller | 4", DS40001882L Table 11-3, md line 1856). Kvasir::EIC's lines are
    // sub-interrupts of it (atsam_common/EIC.hpp).
    template<unsigned Line>
    using EicExtIntVector = std::remove_cvref_t<decltype(eic)>;
};

namespace Nvic {
    // the core's reserved slots, then the lines the part's SVD does not name
    template<>
    struct InterruptOffsetTraits<Kvasir::Tag::User>
      : Core::ChipInterruptTraits<Core::InterruptRules,
                                  Peripheral::interruptEnd,
                                  Peripheral::interruptGaps> {};
}   // namespace Nvic
}   // namespace Kvasir
