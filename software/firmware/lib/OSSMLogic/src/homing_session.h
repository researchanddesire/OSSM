#pragma once

#include <cstdint>

namespace homing_session {

struct Ticket {
    uint64_t generation = 0;
};

// Callers serialize this gate with the same lock as arm/state publication.
// A ticket belongs to the original request, including time spent waiting for
// task memory, network shutdown, ADC setup, or persistent writes.
class Gate {
public:
    Ticket begin() {
        advance();
        active_ = true;
        return {generation_};
    }
    void cancel() {
        active_ = false;
        advance();
    }
    bool accepts(Ticket ticket) const {
        return active_ && ticket.generation != 0 &&
               ticket.generation == generation_;
    }
    bool active() const { return active_; }
    void beginCancel() { cancel(); ++pendingFlushes_; }
    void finishCancel() { if (pendingFlushes_ != 0) --pendingFlushes_; }
    bool commandsReady() const { return pendingFlushes_ == 0; }
    template <class Action> bool commit(Ticket ticket, Action action) {
        return accepts(ticket) && action();
    }
private:
    void advance() { if (++generation_ == 0) ++generation_; }
    uint64_t generation_ = 0;
    bool active_ = false;
    uint32_t pendingFlushes_ = 0;
};

}  // namespace homing_session
