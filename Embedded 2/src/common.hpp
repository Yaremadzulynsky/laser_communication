#include "defines.hpp"
#include "esp_task_wdt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
 #include "driver/gpio.h"
#include "esp_timer.h"