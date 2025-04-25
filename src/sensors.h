#pragma once

#include <Arduino.h>
#include <ESP32Encoder.h>
#include "EncoderReader.h"
#include "MecanumDrive.h"

// Initialize encoder hardware and reader
void initEncoders();

// Read raw encoder counts for all wheels
// Returns a struct with counts: frontLeft, frontRight, rearLeft, rearRight
EncoderData readEncoders();

// Read calculated RPMs for all wheels
// Returns a struct with RPMs: frontLeft, frontRight, rearLeft, rearRight
RPMData getRPMs();
