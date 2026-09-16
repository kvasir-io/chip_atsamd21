#pragma once

#include "Sercom_I2C.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Register/Apply.hpp"
#include "kvasir/Util/StaticFunction.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Sercom { namespace I2C {

    /// The queued I2C master for the SAMD SERCOM, with the same interface as the RP2040 and
    /// RP2350 `Kvasir::I2C::I2CBehaviorQueued`: a request carries its own buffers and a
    /// callback, the bus owns the sequencing, and completion arrives in the ISR. That is the
    /// interface `Kvasir::I2C::Device` -- and so every chip
    /// description in kvasir_devices -- is written against, so this is what lets a SAMD board
    /// use them.
    ///
    /// Sercom_I2C.hpp keeps the SERCOM's register configuration (Detail::I2CBase) this is
    /// built on; the older non-queued `I2CBehavior` that lived there went on 2026-09-13
    /// with the last driver written against its acquire/operationState style. What this
    /// one guarantees the engine:
    ///
    ///  * a NAK is reported as `notAcknowledged` and a bus error as `failed`. The engine
    ///    counts only NAKs towards declaring a device absent, so this is what makes
    ///    parking and probing work;
    ///  * the payload is read straight out of `sendData` and written straight into
    ///    `receiveData`, so a transfer is not limited by an internal buffer;
    ///  * requests queue, so a device can submit from inside another's callback.
    ///
    /// One transfer is on the wire at a time, as the peripheral allows.
    enum class I2CRequestResult : std::uint8_t { failed, notAcknowledged, succeeded };

    template<std::size_t CallbackSize>
    struct I2CRequest {
        std::uint8_t                                         address{};
        std::span<std::byte const>                           sendData{};
        std::span<std::byte>                                 receiveData{};
        StaticFunction<void(I2CRequestResult), CallbackSize> callback{};
    };

    template<typename I2CConfig,
             typename Clock,
             std::size_t QueueDepth_   = 8,
             std::size_t CallbackSize_ = 16>
    struct I2CBehaviorQueued : Detail::I2CBase<I2CConfig> {
        static constexpr std::size_t QueueDepth   = QueueDepth_;
        static constexpr std::size_t CallbackSize = CallbackSize_;
        /// What the bus is clocked at, re-exported from the config so a Kvasir::I2C::Bus can
        /// report its periodic traffic as a fraction of the bandwidth rather than a bit rate.
        static constexpr auto BaudRate = I2CConfig::baudRate;

        using base    = Detail::I2CBase<I2CConfig>;
        using Regs    = typename base::Regs;
        using tp      = typename Clock::time_point;
        using Request = I2CRequest<CallbackSize>;
        using Result  = I2CRequestResult;

        /// The transfer time the peripheral is given before the transaction is abandoned.
        /// Generous: at 100 kHz a 32-byte transfer is about 3 ms.
        static constexpr auto Timeout = std::chrono::milliseconds{100};

        static void runtimeInit() { apply(write(Regs::STATUS::BUSSTATEValC::idle)); }

        static void reset() {
            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            active_ = false;
            state_  = State::idle;
            drainQueueWithFailure_();
            apply(set(Regs::CTRLA::swrst));
            while(apply(read(Regs::SYNCBUSY::swrst))) {}
            apply(base::initStepPeripheryConfig);
            apply(base::initStepInterruptConfig);
            apply(base::initStepPeripheryEnable);
            runtimeInit();
        }

        /// False when the queue is full; the caller tries again next turn.
        static bool submit(Request const& req) {
            if(requestQueue_.size() >= requestQueue_.max_size()) { return false; }
            requestQueue_.push(req);

            apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
            if(!active_) { startNext_(); }
            apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
            return true;
        }

        /// Once per main-loop turn per bus. Owns the timeout, and restarts the queue if a
        /// submit happened to lose the race with a completing transfer.
        static void handler() {
            auto const now = Clock::now();

            if(!active_) {
                if(!requestQueue_.empty()) {
                    apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                    if(!active_) { startNext_(); }
                    apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
                }
                return;
            }

            if(now > timeoutTime_) {
                apply(Nvic::makeDisable(typename base::InterruptIndexs{}));
                if(active_ && now > timeoutTime_) {
                    apply(nack_stop);
                    apply(write(Regs::STATUS::BUSSTATEValC::idle));
                    complete_(Result::failed);
                }
                apply(Nvic::makeEnable(typename base::InterruptIndexs{}));
            }
        }

        static void onIsr() {
            bool const error  = apply(read(Regs::INTFLAG::error));
            bool const rxnack = apply(read(Regs::STATUS::rxnack));

            if(!active_) {
                // A stray interrupt with nothing running: put the bus back to idle.
                apply(write(Regs::STATUS::BUSSTATEValC::idle));
                clearFlags_();
                return;
            }

            if(error || rxnack) {
                // rxnack alone is the device not answering, which is what tells a driver the
                // part is absent; anything else is a bus fault and says nothing about it.
                apply(nack_stop);
                complete_(rxnack && !error ? Result::notAcknowledged : Result::failed);
                clearFlags_();
                return;
            }

            if(state_ == State::sending) {
                if(sendIndex_ < currentRequest_.sendData.size()) {
                    apply(write(Regs::DATA::data,
                                static_cast<std::uint8_t>(currentRequest_.sendData[sendIndex_])));
                    ++sendIndex_;
                } else if(!currentRequest_.receiveData.empty()) {
                    // repeated START into the read phase
                    state_ = State::receiving;
                    apply(
                      write(Regs::ADDR::addr, (unsigned(currentRequest_.address) << 1U) | 0x01U));
                } else {
                    apply(ack_stop);
                    complete_(Result::succeeded);
                }
            } else {   // receiving
                currentRequest_.receiveData[receivedCount_]
                  = static_cast<std::byte>(Kvasir::Register::get<0>(apply(read(Regs::DATA::data))));
                ++receivedCount_;
                if(receivedCount_ < currentRequest_.receiveData.size()) {
                    apply(ack_byte_read);
                } else {
                    apply(nack_stop);
                    complete_(Result::succeeded);
                }
            }
            clearFlags_();
        }

        template<typename... Ts>
        static constexpr auto makeIsr(brigand::list<Ts...>) {
            return brigand::list<
              Kvasir::Nvic::Isr<std::addressof(onIsr), Nvic::Index<Ts::value>>...>{};
        }

        using Isr = decltype(makeIsr(typename base::InterruptIndexs{}));

    private:
        enum class State : std::uint8_t { idle, sending, receiving };

        static constexpr auto ack_byte_read
          = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_ack),
                                               write(Regs::CTRLB::CMDValC::ack_byte_read)));
        static constexpr auto nack_stop
          = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_nack),
                                               write(Regs::CTRLB::CMDValC::ack_stop)));
        static constexpr auto ack_stop
          = list(Regs::CTRLB::overrideDefaults(write(Regs::CTRLB::ACKACTValC::send_ack),
                                               write(Regs::CTRLB::CMDValC::ack_stop)));

        static void clearFlags_() {
            apply(set(Regs::INTFLAG::mb, Regs::INTFLAG::sb, Regs::INTFLAG::error));
        }

        /// Finish the running request, hand the result to its callback, then start whatever
        /// the callback (or anyone else) queued behind it.
        static void complete_(Result r) {
            active_ = false;
            state_  = State::idle;
            auto cb = currentRequest_.callback;
            if(cb) { cb(r); }
            if(!active_) { startNext_(); }
        }

        static void startNext_() {
            if(requestQueue_.empty()) { return; }
            currentRequest_ = requestQueue_.front();
            requestQueue_.pop();

            sendIndex_     = 0;
            receivedCount_ = 0;
            timeoutTime_   = Clock::now() + Timeout;
            active_        = true;

            if(!currentRequest_.sendData.empty()) {
                state_ = State::sending;
                apply(write(Regs::ADDR::addr, unsigned(currentRequest_.address) << 1U));
            } else if(!currentRequest_.receiveData.empty()) {
                state_ = State::receiving;
                apply(write(Regs::ADDR::addr, (unsigned(currentRequest_.address) << 1U) | 0x01U));
            } else {
                // Nothing to transfer: not something a driver asks for, but do not wedge.
                complete_(Result::failed);
            }
        }

        static void drainQueueWithFailure_() {
            while(!requestQueue_.empty()) {
                auto req = requestQueue_.front();
                requestQueue_.pop();
                if(req.callback) { req.callback(Result::failed); }
            }
        }

        inline static Kvasir::Atomic::Queue<Request, QueueDepth> requestQueue_{};
        inline static Request                                    currentRequest_{};
        inline static bool                                       active_{false};
        inline static State                                      state_{State::idle};
        inline static std::size_t                                sendIndex_{0};
        inline static std::size_t                                receivedCount_{0};
        inline static tp                                         timeoutTime_{};
    };

}}}   // namespace Kvasir::Sercom::I2C
