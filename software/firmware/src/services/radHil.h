#pragma once

// Observe normal application work. These functions do not advance state,
// suppress updates, actuate hardware, or alter persistent settings.
void radHilStart();
void radHilProgress(bool idleAndReady);

#if defined(RAD_HIL_VARIANT) && defined(ARDUINO)
void radHilStateChanged(const char* state);
void radHilBleConnected(bool connected);
#else
inline void radHilStateChanged(const char*) {}
inline void radHilBleConnected(bool) {}
#endif
