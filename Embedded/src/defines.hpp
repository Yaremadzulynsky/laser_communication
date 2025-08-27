// Public configuration and logging macros shared by TX and RX
#pragma once

// ================== TIMING CONFIG ==================
// Bit duration in milliseconds; RX lock reliability goes down as this value decreases.
#define BIT_MS                 (14)
// Minimum time between accepted rising edges (software debounce), in microseconds.
// This value should never be greater than the bit duration
#define RX_DEBOUNCE_US         ((10) * 1000)

// ================== FRAME/PROTOCOL CONFIG ==================
#define MAX_PAYLOAD_LENGTH     (128)
#define HALF_MS                (BIT_MS / 2)
#define START_OF_FRAME_BYTE    (0xFE) // 11111110
#define END_OF_FRAME_BYTE      (0xFD) // 11111101
#define PREAMBLE_REPS          (2)
#define PREAMBLE_BYTE          (0xFF) // 11111111
#define FREQUENCY_TOLERANCE_US (4000)

// ================== FEATURE FLAGS ==================
#define ENABLE_RX              (1)
#define ENABLE_TX              (1)

// ================== LOGGING ==================
#define DEBUG_RX               (0)
#define DEBUG_TX               (0)

// Logging macros inline tag strings to avoid unused-variable warnings when DEBUG_* == 0

#if DEBUG_RX
#define LOG_RX(format, ...) ESP_LOGI("LASER_RX", format, ##__VA_ARGS__)
#else
#define LOG_RX(format, ...) ((void)0)
#endif

#if DEBUG_TX
#define LOG_TX(format, ...) ESP_LOGI("LASER_TX", format, ##__VA_ARGS__)
#else
#define LOG_TX(format, ...) ((void)0)
#endif