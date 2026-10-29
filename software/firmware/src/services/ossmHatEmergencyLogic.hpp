#pragma once

#include "ossmHatEmergency.h"

// Pure deadline policy; the ISR adapter alone owns physical routing/FIFOs.
// Always inline: no out-of-line flash call is permitted from the raw ISR.
namespace ossm_hat_emergency {
#define HAT_IRQ_INLINE inline __attribute__((always_inline))
constexpr uint32_t guardDeadlineUs = 34000;
// Anticipatory: preserve the guard's100ms freshness contract while reserving
// nominal IRQ/RTU/DE/transmission margin under bounded interrupt masking.
constexpr uint32_t telemetryDeadlineUs = 80000;
constexpr uint32_t stopDeadlineUs = 50000;
constexpr uint32_t quietUs = 3000;
constexpr uint32_t setupUs = 1000;
constexpr uint32_t txTimeoutUs = 3000;
constexpr uint32_t echoTimeoutUs = 100000;

HAT_IRQ_INLINE bool expected(OssmHatEmergencyCause cause) {
    return cause == OssmHatEmergencyCause::OffOnlyPreflight ||
        cause == OssmHatEmergencyCause::CacheOffPreflight ||
        cause == OssmHatEmergencyCause::Qualification;
}

enum class Phase : uint32_t { Ready, Quiet, Setup, Transmit, Echo, Verify, Failed };
enum Action : uint32_t { NoAction=0, Isolate=1, EnableDirection=2,
                         WriteOff=4, DisableDirection=8 };
HAT_IRQ_INLINE bool verificationReadAllowed(Phase phase, bool isolated,
    bool done, bool echo, bool busy, bool protocolFailure) {
    return phase==Phase::Verify && isolated && done && echo && !busy && !protocolFailure;
}
struct Inputs {
    bool outputPossible = false;
    bool fault = false;
    bool stopPending = false;
    bool rxIdle = false;
    bool txIdle = false;
    uint32_t guardAt = 0;
    uint32_t telemetryAt = 0;
    uint32_t stopAt = 0;
    uint32_t lastRxAt = 0;
};

class Deadline {
  public:
    HAT_IRQ_INLINE bool schedule(OssmHatEmergencyCause cause, uint32_t now) {
        if (phase != Phase::Ready || !expected(cause)) return false;
        scheduled = cause;
        scheduledAt = now;
        return true;
    }

    HAT_IRQ_INLINE uint32_t tick(uint32_t now, const Inputs &in) {
        if (phase == Phase::Ready) {
            OssmHatEmergencyCause reason = OssmHatEmergencyCause::None;
            if (in.outputPossible && in.fault) reason = OssmHatEmergencyCause::ExplicitFault;
            else if (scheduled != OssmHatEmergencyCause::None && now-scheduledAt >= 3000)
                reason = scheduled;
            else if (in.outputPossible && now-in.guardAt > guardDeadlineUs)
                reason = OssmHatEmergencyCause::GuardDeadline;
            else if (in.outputPossible && now-in.telemetryAt > telemetryDeadlineUs)
                reason = OssmHatEmergencyCause::TelemetryDeadline;
            else if (in.outputPossible && in.stopPending && now-in.stopAt > stopDeadlineUs)
                reason = OssmHatEmergencyCause::StopTimeout;
            if (reason == OssmHatEmergencyCause::None) return NoAction;
            cause = reason;
            phase = Phase::Quiet;
            trippedAt = stageAt = now;
            return Isolate;
        }
        if (in.fault && expected(cause)) vetoRestore = true;
        if (phase == Phase::Quiet) {
            if (now-stageAt > 50000) { phase=Phase::Failed; return DisableDirection; }
            if (in.rxIdle && in.txIdle && now-in.lastRxAt >= quietUs && now-stageAt >= quietUs) {
                phase=Phase::Setup; stageAt=now; return EnableDirection;
            }
        } else if (phase == Phase::Setup && now-stageAt >= setupUs) {
            if (!in.txIdle) { phase=Phase::Failed; return DisableDirection; }
            phase=Phase::Transmit; stageAt=now; return WriteOff;
        } else if (phase == Phase::Transmit && now-stageAt > txTimeoutUs) {
            phase=Phase::Failed; return DisableDirection;
        } else if (phase == Phase::Echo && now-stageAt > echoTimeoutUs) {
            phase=Phase::Failed; return DisableDirection;
        }
        return NoAction;
    }

    HAT_IRQ_INLINE bool sent(uint32_t now, bool idle) {
        if (phase != Phase::Transmit || !idle) return false;
        phase=Phase::Echo; stageAt=now; return true;
    }
    HAT_IRQ_INLINE bool echoed() {
        if (phase != Phase::Echo) return false;
        phase=Phase::Verify; return true;
    }
    HAT_IRQ_INLINE bool canRestore(bool parked, bool off, bool quiet, bool fault) const {
        return phase == Phase::Verify && expected(cause) && parked && off && quiet &&
            !fault && !vetoRestore;
    }
    HAT_IRQ_INLINE void fail() { phase=Phase::Failed; vetoRestore=true; }

    Phase phase = Phase::Ready;
    OssmHatEmergencyCause cause = OssmHatEmergencyCause::None;
    bool vetoRestore = false;
    uint32_t trippedAt = 0;
  private:
    OssmHatEmergencyCause scheduled = OssmHatEmergencyCause::None;
    uint32_t scheduledAt = 0;
    uint32_t stageAt = 0;
};

// The quiet/rate proof consumes distinct coherent FC03 samples, never FC06
// echoes or FC7B's immediate vendor value. It is independent of guard policy.
class QuietProof {
  public:
    void reset() { *this = {}; }
    bool accept(uint32_t seq, uint32_t epoch, uint32_t now, int32_t counts,
                bool healthyOff) {
        if (!seq || !epoch || seq == sequence) return false;
        if (!healthyOff || (continuity && continuity != epoch) ||
            (sequence && now-lastAt > 40)) reset();
        sequence=seq; continuity=epoch; lastAt=now;
        if (!healthyOff) return false;
        if (!samples) { firstAt=now; minimum=maximum=counts; }
        if (counts < minimum) minimum=counts;
        if (counts > maximum) maximum=counts;
        if (static_cast<int64_t>(maximum)-minimum > 4) {
            samples=0; firstAt=now; minimum=maximum=counts;
        }
        if (samples < 65535) ++samples;
        const uint32_t window=now-firstAt;
        // At least500ms of actual >=45Hz receipts: intended poll period20ms.
        return samples>=24 && window>=500 &&
            static_cast<uint32_t>(samples-1)*1000U >= 45U*window;
    }
    uint32_t sequence=0, continuity=0, lastAt=0;
  private:
    uint32_t firstAt=0;
    uint16_t samples=0;
    int32_t minimum=0, maximum=0;
};
#undef HAT_IRQ_INLINE
} // namespace ossm_hat_emergency
