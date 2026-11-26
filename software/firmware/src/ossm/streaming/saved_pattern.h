#pragma once

#include "saved_pattern_library.h"

namespace streaming {

void loadSavedPattern();
bool getSavedPattern(SavedPattern &out, int slot = selectedPatternSlot);
bool hasSavedPattern();
PatternCatalog getPatternCatalog();

struct SavePattern {
    const SavedPattern &value;
    bool &written;
    int slot = selectedPatternSlot;
    const char *name = nullptr;
    size_t nameLength = 0;
};
struct SelectPattern {
    int slot;
    bool &written;
};
void storePatternInMenu(const SavePattern &event);
void selectPatternInMenu(const SelectPattern &event);
void toggleWiredPattern();

}  // namespace streaming
