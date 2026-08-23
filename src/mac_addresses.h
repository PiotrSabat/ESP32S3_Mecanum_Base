#ifndef MAC_ADDRESSES_H
#define MAC_ADDRESSES_H

#include <Arduino.h>

// MAC addresses of devices – adjust to your hardware.
// Copy this file to mac_addresses_private.h and fill in the real addresses;
// the private copy is git-ignored, so your hardware details stay local.

// Mecanum platform – ESP32-S3
uint8_t macPlatformMecanum[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

// Gamepad (Pad) – ESP32-S3
uint8_t macPadXiao[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

// Debug monitor – ESP32-LX6
uint8_t macMonitorDebug[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

#endif // MAC_ADDRESSES_H
