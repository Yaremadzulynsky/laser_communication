

// ================== CONFIG ==================
// #define BIT_MS              (10) //64 is max on RMT TX

//I think min(BIT_MS-RX_DEBOUNCE_US) must be < 2500us for the laser to be able to detect the edges
// #define BIT_MS              (12) //Fastest possible message but unreliable locks
// #define RX_DEBOUNCE_US      ((10)*1000)                   // Pair with Fastest possible message but unreliable locks
#define BIT_MS              (14) //Fastest possible message but unreliable locks
#define RX_DEBOUNCE_US      ((10)*1000)                   // Pair with Fastest possible message but unreliable locks

// #define BIT_MS              (10) //min for messages with breaks in between for some reason? 
// #define RX_DEBOUNCE_US      ((6)*1000)                   // Pair with 

#define MAX_PAYLOAD_LENGTH (128)

#define HALF_MS             (BIT_MS/2)
#define START_OF_FRAME_BYTE (0xFE)                    // 11111110
#define END_OF_FRAME_BYTE   (0xFD)                    // 11111101
#define PREAMBLE_REPS       (2)
#define PREAMBLE_BYTE       (0xFF)                    // 11111111
#define DEBUG_RX            (0) 
#define DEBUG_TX            (0)
#define ENABLE_RX           (1)
#define ENABLE_TX           (1)
#define FREQUENCY_TOLERANCE_US (4000)


// ================== LOG TAG_TX =================
static const char *TAG_TX = "LASER_TX";
static const char *TAG_RX = "LASER_RX";

#if DEBUG_RX
#define LOG_RX(format, ...) ESP_LOGI(TAG_RX, format, ##__VA_ARGS__)
#else
#define LOG_RX(format, ...) ((void)0)
#endif

#if DEBUG_TX
#define LOG_TX(format, ...) ESP_LOGI(TAG_TX, format, ##__VA_ARGS__)
#else
#define LOG_TX(format, ...) ((void)0)
#endif

// Minimum time between accepted rising edges (software debounce)
// #define RX_DEBOUNCE_US      ((6)*1000)                   // 20 ms
// #define RX_DEBOUNCE_US      ((20)*1000)                   // Pair with 
// // Additional debounce in RX task for queued edges
// #define RX_TASK_DEBOUNCE_US (0000)                   // 10 ms