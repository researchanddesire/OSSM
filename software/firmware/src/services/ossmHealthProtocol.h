#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

// OSSM_HEALTH: the production health record read by the release hardware
// runner. One JSON object per line, the same events and fields as the
// Lockbox's LKBX_HEALTH, plus the OSSM's HAT fields. No secrets: no tokens,
// Wi-Fi passwords or MQTT credentials.
namespace ossm_health {

struct Boot {
    uint32_t bootId = 0;
    unsigned long long uptimeMs = 0;
    const char *buildSha = "";
    const char *track = "";
    const char *variant = "";   // "4mb" (V1) or "16mb" (V2)
    const char *deviceId = "";  // 12 upper-case hex, as RAD_ID prints it
    const char *imageSha256 = "";
    int resetReason = 0;
    const char *hatMode = "auto";  // "auto" or "off"
};

struct Heartbeat {
    uint32_t bootId = 0;
    unsigned long long uptimeMs = 0;
    bool mainMenu = false;
    uint32_t menuAgeMs = 0;
    bool hat = false;
    bool homed = false;
};

inline size_t complete(int size, size_t capacity) {
    return size > 0 && static_cast<size_t>(size) < capacity ? static_cast<size_t>(size) : 0;
}

inline const char *flag(bool value) { return value ? "true" : "false"; }

// Each record starts on its own line, even after a partial ordinary log line.
inline size_t formatBoot(char *output, size_t capacity, const Boot &boot) {
    return complete(std::snprintf(output, capacity,
        "\nOSSM_HEALTH {\"event\":\"boot\",\"boot_id\":%lu,\"uptime_ms\":%llu,"
        "\"build_sha\":\"%s\",\"track\":\"%s\",\"variant\":\"%s\",\"device_id\":\"%s\","
        "\"image_sha256\":\"%s\",\"reset_reason\":%d,\"hat_mode\":\"%s\"}\n",
        static_cast<unsigned long>(boot.bootId), boot.uptimeMs, boot.buildSha, boot.track,
        boot.variant, boot.deviceId, boot.imageSha256, boot.resetReason, boot.hatMode),
        capacity);
}

inline size_t formatWifiConnected(char *output, size_t capacity, uint32_t bootId,
                                  unsigned long long uptimeMs, const char *ip) {
    return complete(std::snprintf(output, capacity,
        "\nOSSM_HEALTH {\"event\":\"wifi_connected\",\"boot_id\":%lu,\"uptime_ms\":%llu,"
        "\"ip\":\"%s\"}\n",
        static_cast<unsigned long>(bootId), uptimeMs, ip), capacity);
}

inline size_t formatMqttConnected(char *output, size_t capacity, uint32_t bootId,
                                  unsigned long long uptimeMs, const char *broker) {
    return complete(std::snprintf(output, capacity,
        "\nOSSM_HEALTH {\"event\":\"mqtt_connected\",\"boot_id\":%lu,\"uptime_ms\":%llu,"
        "\"broker\":\"%s\"}\n",
        static_cast<unsigned long>(bootId), uptimeMs, broker), capacity);
}

inline size_t formatHeartbeat(char *output, size_t capacity, const Heartbeat &beat) {
    return complete(std::snprintf(output, capacity,
        "\nOSSM_HEALTH {\"event\":\"heartbeat\",\"boot_id\":%lu,\"uptime_ms\":%llu,"
        "\"main_menu\":%s,\"menu_age_ms\":%lu,\"hat\":%s,\"homed\":%s}\n",
        static_cast<unsigned long>(beat.bootId), beat.uptimeMs, flag(beat.mainMenu),
        static_cast<unsigned long>(beat.menuAgeMs), flag(beat.hat), flag(beat.homed)),
        capacity);
}

// The heartbeat reports at most every 5 s, counted from the boot record, so
// consecutive records always show advancing time. Wraps with millis().
constexpr uint32_t heartbeatIntervalMs = 5000;
inline bool heartbeatDue(uint32_t lastMs, uint32_t nowMs) {
    return static_cast<uint32_t>(nowMs - lastMs) >= heartbeatIntervalMs;
}

// The release variant from the image's flash profile.
inline const char *variantForFlash(uint32_t targetFlashBytes) {
    if (targetFlashBytes == 4u * 1024u * 1024u) return "4mb";
    if (targetFlashBytes == 8u * 1024u * 1024u) return "8mb";
    if (targetFlashBytes == 16u * 1024u * 1024u) return "16mb";
    return "unknown";
}

}  // namespace ossm_health
