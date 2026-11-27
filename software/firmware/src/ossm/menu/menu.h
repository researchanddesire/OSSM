#ifndef OSSM_MENU_MENU_H
#define OSSM_MENU_MENU_H

namespace menu {

/**
 * Draw and run the main menu
 * Handles encoder input for menu navigation
 */
void drawMenu();
struct WiredPatternChoice {};
void drawSavedPatterns();
bool confirmSavedPattern();

}  // namespace menu

#endif  // OSSM_MENU_MENU_H
