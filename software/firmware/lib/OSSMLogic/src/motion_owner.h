#pragma once

#include <cstdint>

namespace motion_owner {
using Validator = bool (*)(uint64_t);
struct Ticket {
    uint64_t revision = 0;
    uint32_t binding = 0;
    Validator validator = nullptr;
};

// Caller serializes snapshot/bind/matches. Invoke the external validator only
// after releasing that lock. An unconfigured standalone library remains valid.
class Gate {
public:
    void bind(uint64_t revision, Validator validator) {
        if (++binding_ == 0) ++binding_;
        revision_ = revision;
        validator_ = validator;
    }
    Ticket snapshot() const { return {revision_, binding_, validator_}; }
    bool matches(Ticket ticket) const {
        return ticket.revision == revision_ && ticket.binding == binding_ &&
               ticket.validator == validator_;
    }
private:
    uint64_t revision_ = 0;
    uint32_t binding_ = 0;
    Validator validator_ = nullptr;
};
inline bool authorized(Ticket ticket) {
    return ticket.validator == nullptr || ticket.validator(ticket.revision);
}
}  // namespace motion_owner
