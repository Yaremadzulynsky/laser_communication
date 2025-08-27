// Shared system includes and defines for TX/RX modules
#pragma once

#include "defines.hpp" // project-wide configuration and logging macros

// C stdlib
#include <stdarg.h> // va_list and variadic utilities
#include <stdio.h> // standard I/O (printf, putchar)
#include <stdint.h> // fixed-width integer types
#include <stdbool.h> // C boolean type
#include <string.h> // memcpy, memset, strcmp, etc.

// ESP-IDF core
#include "esp_err.h" // ESP-IDF error types and helpers
#include "esp_log.h" // ESP-IDF logging API
#include "esp_timer.h" // high-resolution microsecond timer

// Drivers
#include "driver/gpio.h" // GPIO configuration and ISR
#include "driver/rmt_encoder.h" // RMT encoder interfaces
#include "driver/rmt_tx.h" // RMT transmit driver
#include "esp_task_wdt.h" // task watchdog control

// FreeRTOS
#include "freertos/FreeRTOS.h" // FreeRTOS core definitions
#include "freertos/queue.h" // FreeRTOS queue API
#include "freertos/task.h" // FreeRTOS task API
