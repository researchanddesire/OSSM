#include "UserConfig.h"

namespace UserConfig {
bool displayMetric = true;
std::atomic<float> afterHomingPositionMm{0.0f};
float mqttPublishFrequencyHz = 30.0f;
}  // namespace UserConfig
