#pragma once

#include <FreeRTOS.h>
#include <task.h>

// Create and pin tasks for ESP-NOW, motor control, PID, and debug display
void createTasks();

// Individual task function prototypes (for reference or unit tests)
void espNowTask(void *parameter);
void motorControlTask(void *parameter);
void pidTask(void *parameter);
void debugTask(void *parameter);
