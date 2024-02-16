#pragma once
#include "Interrupt.hpp"
#include "chip/atsam_common/DMAC.hpp"
#include "chip/atsam_common/Sercom_Common.hpp"
#include "kvasir/Io/Types.hpp"

#include <array>

namespace Kvasir { namespace Sercom { namespace Traits {

    namespace detail {

        template<unsigned Instance>
        struct IsrIndex;
        template<>
        struct IsrIndex<0> {
            using Type = brigand::list<decltype(Kvasir::Interrupt::sercom0)>;
        };
        template<>
        struct IsrIndex<1> {
            using Type = brigand::list<decltype(Kvasir::Interrupt::sercom1)>;
        };
        template<>
        struct IsrIndex<2> {
            using Type = brigand::list<decltype(Kvasir::Interrupt::sercom2)>;
        };
        template<>
        struct IsrIndex<3> {
            using Type = brigand::list<decltype(Kvasir::Interrupt::sercom3)>;
        };
        template<>
        struct IsrIndex<4> {
            using Type = brigand::list<decltype(Kvasir::Interrupt::sercom4)>;
        };
        template<>
        struct IsrIndex<5> {
            using Type = brigand::list<decltype(Kvasir::Interrupt::sercom5)>;
        };
    }   // namespace detail

    struct SercomTraits : SercomTraitsBase<SercomTraits> {
        static constexpr int                sercomInstanceBegin = 0;
        static constexpr int                sercomInstanceEnd   = 6;
        static constexpr std::array<int, 0> disabledInstances   = {{}};

        template<unsigned Instance>
        struct IsrIndex {
            using Type = typename detail::IsrIndex<Instance>::Type;
        };

        template<unsigned Instance>
        static constexpr auto DmaTriggers() {
            static_assert(5 >= Instance);
            if constexpr(Instance == 0) {
                return std::make_pair(
                  Kvasir::DMAC::TriggerSource::sercom0rx,
                  DMAC::TriggerSource::sercom0tx);
            } else if constexpr(Instance == 1) {
                return std::make_pair(
                  Kvasir::DMAC::TriggerSource::sercom1rx,
                  DMAC::TriggerSource::sercom1tx);
            }

            else if constexpr(Instance == 2)
            {
                return std::make_pair(
                  Kvasir::DMAC::TriggerSource::sercom2rx,
                  DMAC::TriggerSource::sercom2tx);
            } else if constexpr(Instance == 3) {
                return std::make_pair(
                  Kvasir::DMAC::TriggerSource::sercom3rx,
                  DMAC::TriggerSource::sercom3tx);
            } else if constexpr(Instance == 4) {
                return std::make_pair(
                  Kvasir::DMAC::TriggerSource::sercom4rx,
                  DMAC::TriggerSource::sercom4tx);
            } else if constexpr(Instance == 5) {
                return std::make_pair(
                  Kvasir::DMAC::TriggerSource::sercom5rx,
                  DMAC::TriggerSource::sercom5tx);
            }
        }
        template<unsigned Instance>
        static constexpr auto DmaRX_Trigger() {
            return DmaTriggers<Instance>().first;
        }
        template<unsigned Instance>
        static constexpr auto DmaTX_Trigger() {
            return DmaTriggers<Instance>().second;
        }

        static constexpr std::array pinMuxInfos{
  // LVariant
  // Instance 0
          PinInfo{0, 0,  4, 3, 0},
          PinInfo{0, 0,  5, 3, 1},
          PinInfo{0, 0,  6, 3, 2},
          PinInfo{0, 0,  7, 3, 3},
          PinInfo{0, 0,  8, 2, 0},
          PinInfo{0, 0,  9, 2, 1},
          PinInfo{0, 0, 10, 2, 2},
          PinInfo{0, 0, 11, 2, 3},
 // Instance 1
          PinInfo{1, 0, 16, 2, 0},
          PinInfo{1, 0, 17, 2, 1},
          PinInfo{1, 0, 18, 2, 2},
          PinInfo{1, 0, 19, 2, 3},
          PinInfo{1, 0, 30, 3, 2},
          PinInfo{1, 0, 31, 3, 3},
 // Instance 2
          PinInfo{2, 0,  8, 3, 0},
          PinInfo{2, 0,  9, 3, 1},
          PinInfo{2, 0, 10, 3, 2},
          PinInfo{2, 0, 11, 3, 3},
          PinInfo{2, 0, 12, 2, 0},
          PinInfo{2, 0, 13, 2, 1},
          PinInfo{2, 0, 14, 2, 2},
          PinInfo{2, 0, 15, 2, 3},
 // Instance 3
          PinInfo{3, 0, 16, 3, 0},
          PinInfo{3, 0, 17, 3, 1},
          PinInfo{3, 0, 18, 3, 2},
          PinInfo{3, 0, 19, 3, 3},
          PinInfo{3, 0, 20, 3, 2},
          PinInfo{3, 0, 21, 3, 3},
          PinInfo{3, 0, 22, 2, 0},
          PinInfo{3, 0, 23, 2, 1},
          PinInfo{3, 0, 24, 2, 2},
          PinInfo{3, 0, 25, 2, 3},
 // Instance 4
          PinInfo{4, 1,  8, 3, 0},
          PinInfo{4, 1,  9, 3, 1},
          PinInfo{4, 1, 10, 3, 2},
          PinInfo{4, 1, 11, 3, 3},
          PinInfo{4, 0, 12, 3, 0},
          PinInfo{4, 0, 13, 3, 1},
          PinInfo{4, 0, 14, 3, 2},
          PinInfo{4, 0, 15, 3, 3},
 // Instance 5
          PinInfo{5, 0, 20, 2, 2},
          PinInfo{5, 0, 21, 2, 3},
          PinInfo{5, 0, 22, 3, 0},
          PinInfo{5, 0, 23, 3, 1},
          PinInfo{5, 0, 24, 3, 2},
          PinInfo{5, 0, 25, 3, 3},
          PinInfo{5, 1,  2, 3, 0},
          PinInfo{5, 1,  3, 3, 1},
        };
    };

}}}   // namespace Kvasir::Sercom::Traits
