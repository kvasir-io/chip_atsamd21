#pragma once

#include "chip/Io.hpp"
#include "chip/PM.hpp"
#include "chip/Sercom_Traits.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Io/Types.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/literals.hpp"
#include "kvasir/Util/using_literals.hpp"
#include "peripherals/SERCOM_I2CM.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <ratio>

namespace Kvasir { namespace Sercom { namespace I2C {
    namespace Detail {

        template<unsigned SercomInstance, int Port, int Pin>
        constexpr bool isValidPinLocationSDA(Kvasir::Register::PinLocation<Port, Pin>) {
            return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(0);
        }

        template<unsigned SercomInstance, int Port, int Pin>
        constexpr bool isValidPinLocationSCL(Kvasir::Register::PinLocation<Port, Pin>) {
            return Traits::SercomTraits::ValidIfPOVal<SercomInstance, Port, Pin>(1);
        }

        template<unsigned SercomInstance, typename SDAPIN>
        struct GetSdaPinConfig;

        template<unsigned SercomInstance, int Port, int Pin>
        struct GetSdaPinConfig<SercomInstance, Kvasir::Register::PinLocation<Port, Pin>> {
            using pinConfig = decltype(action(
              Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<SercomInstance>(
                Register::PinLocation<Port, Pin>{})>{},
              Register::PinLocation<Port, Pin>{}));
        };

        template<unsigned SercomInstance, typename SCLPIN>
        struct GetSclPinConfig;

        template<unsigned SercomInstance, int Port, int Pin>
        struct GetSclPinConfig<SercomInstance, Kvasir::Register::PinLocation<Port, Pin>> {
            using pinConfig = decltype(action(
              Kvasir::Io::Action::PinFunction<Traits::SercomTraits::GetPinFunction<SercomInstance>(
                Register::PinLocation<Port, Pin>{})>{},
              Register::PinLocation<Port, Pin>{}));
        };

        struct BaudConfig {
            unsigned char baud;
            unsigned char baudlow;
            unsigned char hsbaud;
            unsigned char hsbaudlow;
        };

        struct BaudConfigRaw {
            unsigned char baud;
            unsigned char baudlow;
        };

        static constexpr unsigned maxSpeedStandard  = 100'000;
        static constexpr unsigned maxSpeedFast      = 400'000;
        static constexpr unsigned maxSpeedFastPlus  = 1'000'000;
        static constexpr unsigned maxSpeedHighSpeed = 3'400'000;

        static constexpr BaudConfigRaw calcBaudConfigRaw(
          std::uint32_t f_clockSpeed,
          std::uint32_t f_baud,
          double        val,
          double        lowtime,
          double        hightime) {
            double     baudraw = ((double(f_clockSpeed) / double(f_baud)) - val);
            auto const baud    = std::int64_t(
              (baudraw * (hightime / (lowtime + hightime)))   //NOLINT(bugprone-incorrect-roundings)
              + 0.5);
            auto const baudlow = std::int64_t(
              (baudraw * (lowtime / (lowtime + hightime)))
              + 0.5);   //NOLINT(bugprone-incorrect-roundings)
            auto const baudReg = static_cast<unsigned char>(std::clamp<std::int64_t>(baud, 0, 255));
            auto const baudlowReg
              = static_cast<unsigned char>(std::clamp<std::int64_t>(baudlow, 0, 255));
            return {baudReg, baudlowReg};
        }

        constexpr BaudConfig calcBaudConfig(std::uint32_t f_clockSpeed, std::uint32_t f_baud) {
            if(f_baud <= maxSpeedStandard) {
                auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 10, 4.7, 4.0);
                return {raw.baud, raw.baudlow, 0, 0};
            }
            if(f_baud <= maxSpeedFast) {
                auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 10, 1.3, 0.6);
                return {raw.baud, raw.baudlow, 0, 0};
            }
            if(f_baud <= maxSpeedFastPlus) {
                auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 10, 0.5, 0.26);
                return {raw.baud, raw.baudlow, 0, 0};
            }
            auto const raw = calcBaudConfigRaw(f_clockSpeed, f_baud, 2, 2.0, 1.0);
            return {0, 0, raw.baud, raw.baudlow};
        }

        constexpr double calcf_Baud(std::uint32_t f_clockSpeed, BaudConfig baudConfig) {
            if(baudConfig.baud == 0 && baudConfig.baudlow == 0) {
                // high_speed
                if(baudConfig.hsbaudlow == 0) {
                    return double(f_clockSpeed) / (2.0 + 2.0 * double(baudConfig.hsbaud));
                }
                return double(f_clockSpeed)
                     / (2.0 + double(baudConfig.hsbaud) + double(baudConfig.hsbaudlow));
            }
            if(baudConfig.hsbaud == 0 && baudConfig.hsbaudlow == 0) {
                // other
                if(baudConfig.baudlow == 0) {
                    return double(f_clockSpeed) / (10.0 + 2.0 * double(baudConfig.baud));
                }
                return double(f_clockSpeed)
                     / (10.0 + double(baudConfig.baud) + double(baudConfig.baudlow));
            }
            return std::numeric_limits<double>::min();
        }

        template<
          std::uint32_t f_clockSpeed,
          std::uint32_t f_baud,
          std::intmax_t Num,
          std::intmax_t Denom>
        constexpr bool isValidBaudConfig(std::ratio<Num, Denom>) {
            static_assert(f_baud <= maxSpeedHighSpeed, "baudRate to big");

            constexpr auto baudConfig   = calcBaudConfig(f_clockSpeed, f_baud);
            constexpr auto f_baudCalced = calcf_Baud(f_clockSpeed, baudConfig);
            constexpr auto err          = f_baudCalced - double(f_baud);
            constexpr auto absErr       = err > 0.0 ? err : -err;
            constexpr auto ret = absErr <= (double(f_baud) * (double(Num) / (double(Denom))));
            return ret;
        }

        template<typename Regs, unsigned clockSpeed, unsigned baudRate>
        struct GetBaudConfig {
            static constexpr auto baudConfig = calcBaudConfig(clockSpeed, baudRate);

            using config = decltype(list(
              write(Regs::BAUD::baud, Register::value<unsigned char, baudConfig.baud>()),
              write(Regs::BAUD::baudlow, Register::value<unsigned char, baudConfig.baudlow>()),
              write(Regs::BAUD::hsbaud, Register::value<unsigned char, baudConfig.hsbaud>()),
              write(
                Regs::BAUD::hsbaudlow,
                Register::value<unsigned char, baudConfig.hsbaudlow>())));
        };

        template<typename Regs, unsigned baudrate>
        constexpr auto getSpeedConfig() {
            if constexpr(baudrate <= maxSpeedFast) {
                return Regs::CTRLA::SPEEDValC::standard_and_fast_mode;
            } else if constexpr(baudrate <= maxSpeedFastPlus) {
                return Regs::CTRLA::SPEEDValC::fastplus_mode;
            } else {
                return Regs::CTRLA::SPEEDValC::high_speed_mode;
            }
        }

        template<typename I2CConfig_>
        struct I2CBase {
            struct I2CConfig : I2CConfig_ {
                static constexpr auto userConfigOverride = [] {
                    if constexpr(requires { I2CConfig_::userConfigOverride; }) {
                        return I2CConfig_::userConfigOverride;
                    } else {
                        return brigand::list<>{};
                    }
                }();

                static constexpr auto maxBaudRateError = [] {
                    if constexpr(requires { I2CConfig_::maxBaudRateError; }) {
                        return I2CConfig_::maxBaudRateError;
                    } else {
                        return std::ratio<1, 100>{};
                    }
                }();
            };

            // needed config
            // clockSpeed
            // baudRate
            // maxBaudRateError
            // i2cInstance
            // SdaPinLocation
            // SclPinLocation
            // userConfigOverride
            static constexpr auto Instance = I2CConfig::instance;
            using Regs                     = Kvasir::Peripheral::SERCOM_I2CM::Registers<Instance>;

            using InterruptIndexs = decltype(Traits::SercomTraits::getSercomIsrIndexs<Instance>());

            static_assert(
              isValidBaudConfig<I2CConfig::clockSpeed, I2CConfig::baudRate>(
                I2CConfig::maxBaudRateError),
              "invalid baud configuration baudRate error to big");
            static_assert(
              isValidPinLocationSDA<Instance>(I2CConfig::sdaPinLocation),
              "invalid SDAPin");
            static_assert(
              isValidPinLocationSCL<Instance>(I2CConfig::sclPinLocation),
              "invalid SCLPin");

            static constexpr auto powerClockEnable
              = list(typename PM::enable<Regs::baseAddr>::action{});

            static constexpr auto initStepPinConfig = list(
              typename GetSdaPinConfig<
                Instance,
                std::decay_t<decltype(I2CConfig::sdaPinLocation)>>::pinConfig{},
              typename GetSclPinConfig<
                Instance,
                std::decay_t<decltype(I2CConfig::sclPinLocation)>>::pinConfig{});

            static constexpr auto initStepPeripheryConfig = list(
              Regs::BAUD::overrideDefaults(
                typename GetBaudConfig<Regs, I2CConfig::clockSpeed, I2CConfig::baudRate>::config{}),

              Regs::CTRLA::overrideDefaults(
                write(Regs::CTRLA::MODEValC::i2c_master),
                write(getSpeedConfig<Regs, I2CConfig::baudRate>())),
              set(Regs::INTENSET::mb),
              set(Regs::INTENSET::sb),
              set(Regs::INTENSET::error),
              I2CConfig::userConfigOverride);

            static constexpr auto initStepInterruptConfig = list(
              Nvic::makeSetPriority<I2CConfig::isrPriority>(InterruptIndexs{}),
              Nvic::makeClearPending(InterruptIndexs{}));

            static constexpr auto initStepPeripheryEnable
              = list(set(Regs::CTRLA::enable), Nvic::makeEnable(InterruptIndexs{}));
        };
    }   // namespace Detail

    template<typename I2CConfig, typename Clock, std::size_t BufferSize_>
    struct I2CBehavior : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t BufferSize = BufferSize_;
        using base                              = Detail::I2CBase<I2CConfig>;
        using Regs                              = typename base::Regs;
        using tp                                = typename Clock::time_point;

        enum class State { idle, blocked, sending, receiveing };
        enum class OperationState { succeeded, failed, ongoing };
        inline static std::atomic<State>          state_{State::idle};
        inline static std::atomic<std::uint8_t>   receiveSize_{0};
        inline static std::atomic<OperationState> operationState_{OperationState::succeeded};
        inline static Kvasir::Atomic::Queue<std::byte, BufferSize> buffer_{};
        inline static bool                                         stop{false};
        inline static uint8_t                                      addr{0};
        inline static tp                                           timeoutTime{};

        static constexpr auto ack_byte_read = list(Regs::CTRLB::overrideDefaults(
          write(Regs::CTRLB::ACKACTValC::send_ack),
          write(Regs::CTRLB::CMDValC::ack_byte_read)));

        static constexpr auto nack_stop = list(Regs::CTRLB::overrideDefaults(
          write(Regs::CTRLB::ACKACTValC::send_nack),
          write(Regs::CTRLB::CMDValC::ack_stop)));

        static constexpr auto ack_stop = list(Regs::CTRLB::overrideDefaults(
          write(Regs::CTRLB::ACKACTValC::send_ack),
          write(Regs::CTRLB::CMDValC::ack_stop)));

        static void runtimeInit() { apply(write(Regs::STATUS::BUSSTATEValC::idle)); }
        // INTERFACE
        // TODO timeout
        static void reset() {
            apply(makeDisable(typename base::InterruptIndexs{}));
            state_.store(State::idle, std::memory_order_relaxed);
            operationState_.store(OperationState::succeeded, std::memory_order_relaxed);
            apply(set(Regs::CTRLA::swrst));
            while(apply(read(Regs::SYNCBUSY::swrst))) {
            }
            apply(base::initStepPeripheryConfig);
            apply(base::initStepInterruptConfig);
            apply(base::initStepPeripheryEnable);
            runtimeInit();
        }

        template<typename C>
        static void getReceivedBytes(C& c) {
            assert(c.size() <= buffer_.size());
            buffer_.pop_into(c);
        }

        template<typename OIT>
        static void getReceivedBytes(OIT first, OIT last) {
            while(first != last) {
                assert(!buffer_.empty());
                *first = buffer_.front();
                buffer_.pop();
                ++first;
            }
        }

        static OperationState operationState(tp const& currentTime) {
            auto op = operationState_.load(std::memory_order_relaxed);
            if(op == OperationState::ongoing) {
                if(currentTime > timeoutTime) {
                    state_.store(State::blocked, std::memory_order_relaxed);
                    return OperationState::failed;
                }
            }
            return op;
        }

        static bool acquire() {
            if(state_.load(std::memory_order_relaxed) == State::idle) {
                state_.store(State::blocked, std::memory_order_relaxed);
                return true;
            }
            return false;
        }
        static void release() {
            assert(state_.load(std::memory_order_relaxed) != State::idle);   // TODO
            state_.store(State::idle, std::memory_order_relaxed);
        }

        template<typename C>
        static void send(tp const& currentTime, std::uint8_t address, C const& c) {
            /*            K_ASSERT(
              state_.load(std::memory_order_relaxed)
              == Kvasir::none_of(State::sending, State::receiveing));*/
            buffer_.clear();
            buffer_.push(c);
            state_.store(State::sending, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = true;
            timeoutTime = currentTime + 100ms;   // TODO baud*size
            apply(write(Regs::ADDR::addr, unsigned(address) << 1U));
        }

        static void receive(tp const& currentTime, std::uint8_t address, std::uint8_t size) {
            /*            K_ASSERT(
              state_.load(std::memory_order_relaxed)
              == Kvasir::none_of(State::sending, State::receiveing));*/
            assert(size <= buffer_.max_size());
            buffer_.clear();
            receiveSize_.store(size, std::memory_order_relaxed);
            state_.store(State::receiveing, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = true;
            timeoutTime = currentTime + 100ms;   // TODO baud*size
            apply(write(Regs::ADDR::addr, (unsigned(address) << 1U) | 0x01U));
        }

        template<typename C>
        static void
        send_receive(tp const& currentTime, std::uint8_t address, C const& c, std::uint8_t size) {
            /*     K_ASSERT(
              state_.load(std::memory_order_relaxed)
              == Kvasir::none_of(State::sending, State::receiveing));*/
            assert(size <= buffer_.max_size());
            buffer_.clear();
            buffer_.push(c);
            receiveSize_.store(size, std::memory_order_relaxed);
            state_.store(State::sending, std::memory_order_relaxed);
            operationState_.store(OperationState::ongoing, std::memory_order_relaxed);
            stop        = false;
            addr        = address;
            timeoutTime = currentTime + 100ms;   // TODO baud*size
            apply(write(Regs::ADDR::addr, unsigned(address) << 1U));
        }

        // ISR
        static void onIsr() {
            bool const error  = apply(read(Regs::INTFLAG::error));
            bool const rxnack = apply(read(Regs::STATUS::rxnack));

            auto lstate  = state_.load(std::memory_order_relaxed);
            auto lostate = operationState_.load(std::memory_order_relaxed);

            // TODO K_ASSERT(lstate != State::blocked && lstate != State::idle);

            // TODO operation state release
            if(lstate == State::sending) {
                if(error || rxnack) {
                    lstate  = State::blocked;
                    lostate = OperationState::failed;
                    apply(nack_stop);
                } else {
                    if(!buffer_.empty()) {
                        apply(write(Regs::DATA::data, static_cast<std::uint8_t>(buffer_.front())));
                        buffer_.pop();
                    } else {
                        if(stop) {
                            apply(ack_stop);
                            lstate  = State::blocked;
                            lostate = OperationState::succeeded;
                        } else {
                            lstate = State::receiveing;
                            apply(write(Regs::ADDR::addr, (unsigned(addr) << 1U) | 0x01U));
                        }
                    }
                }
            } else if(lstate == State::receiveing) {
                if(error || rxnack) {
                    lstate  = State::blocked;
                    lostate = OperationState::failed;
                    apply(nack_stop);
                } else {
                    auto lreceiveSize = receiveSize_.load(std::memory_order_relaxed);
                    buffer_.push(static_cast<std::byte>(
                      Kvasir::Register::get<0>(apply(read(Regs::DATA::data)))));
                    if(buffer_.size() != lreceiveSize) {
                        apply(ack_byte_read);
                    } else {
                        apply(nack_stop);
                        lstate  = State::blocked;
                        lostate = OperationState::succeeded;
                    }
                }
            } else {
                apply(write(Regs::STATUS::BUSSTATEValC::idle));
            }

            operationState_.store(lostate, std::memory_order_relaxed);
            state_.store(lstate, std::memory_order_relaxed);
            apply(set(Regs::INTFLAG::mb, Regs::INTFLAG::sb, Regs::INTFLAG::error));
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }
        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));
    };

}}}   // namespace Kvasir::Sercom::I2C
