#pragma once
#include "kvasir/Common/Interrupt.hpp"

#include <array>

namespace Kvasir {
namespace Interrupt {
    template<int I>
    using Type = ::Kvasir::Nvic::Index<I>;

    static constexpr Type<-14> nonMaskableInt{};
    static constexpr Type<-13> hardFault{};
    static constexpr Type<-5>  sVCall{};
    static constexpr Type<-2>  pendSV{};
    static constexpr Type<-1>  systick{};
    static constexpr Type<0>   pm{};
    static constexpr Type<1>   sysctl{};
    static constexpr Type<2>   wdt{};
    static constexpr Type<3>   rtc{};
    static constexpr Type<4>   eic{};
    static constexpr Type<5>   nvmctrl{};
    static constexpr Type<6>   dmac{};
    // 7
    static constexpr Type<8>  evsys{};
    static constexpr Type<9>  sercom0{};
    static constexpr Type<10> sercom1{};
    static constexpr Type<11> sercom2{};
    static constexpr Type<12> sercom3{};
    static constexpr Type<13> sercom4{};
    static constexpr Type<14> sercom5{};
    static constexpr Type<15> tcc0{};
    static constexpr Type<16> tcc1{};
    static constexpr Type<17> tcc2{};
    static constexpr Type<18> tc3{};
    static constexpr Type<19> tc4{};
    static constexpr Type<20> tc5{};
    static constexpr Type<21> tc6{};
    static constexpr Type<22> tc7{};
    static constexpr Type<23> adc{};
    static constexpr Type<24> ac{};
    static constexpr Type<25> dac{};
    // 26
    // 27
    static constexpr Type<28> ac1{};
}   // namespace Interrupt

namespace Nvic {
    using namespace Kvasir::Interrupt;
    template<>
    struct InterruptOffsetTraits<void> {
        static constexpr int        begin = -14;
        static constexpr int        end   = 29;
        static constexpr std::array disabled
          = {-13, -12, -11, -10, -9, -8, -7, -6, -4, -3, 7, 26, 27};
        static constexpr std::array noEnable
          = {nonMaskableInt.index(), sVCall.index(), pendSV.index()};
        static constexpr std::array noDisable
          = {nonMaskableInt.index(), sVCall.index(), pendSV.index()};
        static constexpr std::array noSetPending = {sVCall.index(), hardFault.index()};
        static constexpr std::array noClearPending
          = {nonMaskableInt.index(), sVCall.index(), hardFault.index()};
        static constexpr std::array noSetPriority = {nonMaskableInt.index(), hardFault.index()};

        using FaultInterruptIndexs           = brigand::list<decltype(hardFault)>;
        using FaultInterruptIndexsNeedEnable = brigand::list<>;
    };

}   // namespace Nvic
}   // namespace Kvasir
