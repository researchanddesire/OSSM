#ifndef OSSM_COMMUNICATION_MOTION_LINK_H
#define OSSM_COMMUNICATION_MOTION_LINK_H

#include <cstdint>
#include <string>

// Which BLE connection is driving motion, and whether a disconnect must
// stop it. Several centrals can be connected at once (a RADR plus the RAD
// App or the dashboard). Motion used to stop only when the last one left,
// so a RADR that dropped while the app stayed connected left the OSSM
// running at the RADR's speed with nobody holding the controls.
namespace motion_link {

constexpr uint16_t kNoConnection = 0xffff;

// Commands that start or change motion. Stops (go:menu) and network jobs
// do not take ownership.
inline bool isMotionCommand(const std::string &command) {
    return command == "go:strokeEngine" || command == "go:simplePenetration" ||
           command == "go:streaming" ||
           (command.rfind("set:", 0) == 0 && command.rfind("set:wifi:", 0) != 0) ||
           command.rfind("stream:", 0) == 0;
}

class Owner {
  public:
    // A motion command arrived on `connection` (kNoConnection when the
    // transport cannot say which, as for RAD BLE requests).
    void onMotionCommand(uint16_t connection) { owner_ = connection; }

    // `connection` dropped and `remaining` links stay. True when motion
    // must stop: no link is left, or the dropped one commanded motion last.
    bool onDisconnect(uint16_t connection, int remaining) {
        const bool wasOwner = owner_ != kNoConnection && owner_ == connection;
        if (wasOwner || remaining <= 0) owner_ = kNoConnection;
        return remaining <= 0 || wasOwner;
    }

    uint16_t owner() const { return owner_; }

  private:
    uint16_t owner_ = kNoConnection;
};

}  // namespace motion_link

#endif  // OSSM_COMMUNICATION_MOTION_LINK_H
