#pragma once

#include <cstddef>
#include <cstdint>

#include "drawn_waveform.h"

namespace streaming {

// One versioned NVS value. Replacing it commits the period and curve together.
struct SavedPattern {
    uint32_t version = 1;
    uint32_t periodMs = 0;
    uint32_t length = 0;
    char samples[drawn_pattern::maxSamples * 2] = {};
};

bool makeSavedPattern(SavedPattern &out, const char *samples, size_t length,
                      uint32_t periodMs);
void loadSavedPattern();
bool getSavedPattern(SavedPattern &out);
bool hasSavedPattern();
struct SavePattern {
    const SavedPattern &value;
    bool &written;
};
void storePatternInMenu(const SavePattern &event);
void toggleWiredPattern();

}  // namespace streaming
