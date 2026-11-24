#pragma once

// Call configure before Serial.begin, then start once the UART is available.
void configureSerialIdentityUsb();
void startSerialIdentity();
// The factory MAC as 12 upper-case hex digits, the device_id in RAD_ID.
void serialIdentityDeviceId(char (&deviceId)[13]);
#ifdef VERSIONDEV
void setSerialDevelopmentHandler(void (*command)(const char*), void (*poll)());
#endif
