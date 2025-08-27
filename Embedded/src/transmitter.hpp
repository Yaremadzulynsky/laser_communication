#include "common.hpp" // shared includes (ESP-IDF, FreeRTOS) and project defines


// NOTE: In memory bytes 0..127 are transmitted as 128..255 for reliability; RX subtracts 128.
struct tx_data_t
{
	uint8_t data[MAX_PAYLOAD_LENGTH]; // 128 bytes
	size_t len;						  // length of the data
};

// ================== MODULE STATE ==================
static constexpr gpio_num_t output_GPIO = GPIO_NUM_13; // TX pin
static QueueHandle_t g_tx_queue = nullptr; // TX queue
static constexpr size_t TX_QUEUE_LEN = 512; // adjust as needed

// RMT handles
static rmt_channel_handle_t s_rmt_tx_chan = nullptr;
static rmt_encoder_handle_t s_rmt_copy_enc = nullptr;

static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000; // 1 MHz -> 1 tick = 1 us


// ================== RZ HELPER FUNCTIONS ==================
/**
 * @brief Send a single RZ-encoded bit.
 * RZ: bit 1 → HIGH then LOW (half/half); bit 0 → LOW for full bit.
 */
static inline void rz_send_bit(int bit)
{
	// Emit one RZ-encoded bit via RMT. Resolution is 1us.
	const uint32_t half_us = (uint32_t)HALF_MS * 1000U; // fits within 15-bit duration (<= 32767)
	rmt_symbol_word_t sym{};
	if (bit)
	{
		// HIGH for first half, then LOW for second half
		sym.level0 = 1;
		sym.duration0 = (uint16_t)half_us;
		sym.level1 = 0;
		sym.duration1 = (uint16_t)half_us;
	}
	else
	{
		// LOW for entire bit period -> two consecutive LOW halves
		sym.level0 = 0;
		sym.duration0 = (uint16_t)half_us;
		sym.level1 = 0;
		sym.duration1 = (uint16_t)half_us;
	}

	rmt_transmit_config_t tx_cfg{}; // no loop
	ESP_ERROR_CHECK(rmt_transmit(s_rmt_tx_chan, s_rmt_copy_enc, &sym, sizeof(sym), &tx_cfg));
	ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_rmt_tx_chan, -1));
}

/**
 * @brief Send one byte via RZ encoding, LSB-first.
 */
static inline void rz_send_byte(uint8_t value)
{
	rz_send_bit(1);
	for (int i = 0; i < 8; ++i)
	{
		int bit = (value >> i) & 1;
		rz_send_bit(bit);
	}
}

/**
 * @brief Send one frame: PREAMBLE xN, SOF, LEN, PAYLOAD, EOF.
 * Each byte is preceded by a 1 to create a rising edge for phase.
 */
static void rz_send_frame(const uint8_t *payload, size_t len)
{
	// Send preamble (PREAMBLE_REPS times)
	for (int i = 0; i < PREAMBLE_REPS; ++i)
	{
		rz_send_byte(PREAMBLE_BYTE);
	}

	// Send start of frame byte (START_OF_FRAME_BYTE)
	rz_send_byte(START_OF_FRAME_BYTE);

	// Send length of payload (offset by 128)
	uint8_t bytes_to_send = (uint8_t)len + 128;
	rz_send_byte(bytes_to_send);

	// Send payload
	for (size_t i = 0; i < len; ++i)
	{
		rz_send_byte(payload[i] + 128);
	}

	
	rz_send_byte(END_OF_FRAME_BYTE);
}

// ================== TX API ==================
/**
 * @brief Enqueue a payload to be sent by the TX task as a frame.
 * Returns ESP_ERR_INVALID_STATE if queue not ready, ESP_ERR_TIMEOUT if full.
 */
static inline esp_err_t laser_tx_write(tx_data_t tx_data, TickType_t wait_ticks)
{
	// Check if the TX queue is initialized
	if (!g_tx_queue)
		return ESP_ERR_INVALID_STATE;

	// Enqueue payload
	if (xQueueSend(g_tx_queue, &tx_data, wait_ticks) != pdTRUE)
	{
		return ESP_ERR_TIMEOUT;
	}
	return ESP_OK;
}


/**
 * @brief TX task: drain queue and send frames via RZ encoding.
 */
static void tx_task(void *arg)
{
	(void)arg;
	tx_data_t tx_data;
	for (;;)
	{
		// Block until at least one byte is available
		if (xQueueReceive(g_tx_queue, &tx_data, portMAX_DELAY) == pdTRUE)
		{
			// Send the queued payload as a frame
			rz_send_frame(tx_data.data, tx_data.len);

			// Drain any immediately available payloads without blocking
			while (xQueueReceive(g_tx_queue, &tx_data, 0) == pdTRUE)
			{
				rz_send_frame(tx_data.data, tx_data.len);
			}
		}
	}
}


// ================== INIT & TASK ==================
/**
 * @brief Initialize the laser TX GPIO, create a TX queue and tx_task.
 *
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise.
 */
static esp_err_t laser_tx_init(void)
{
	// Create and enable RMT TX channel on LASER_GPIO
	rmt_tx_channel_config_t tx_chan_cfg{};
	tx_chan_cfg.gpio_num = output_GPIO;
	tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
	tx_chan_cfg.resolution_hz = RMT_RESOLUTION_HZ; // 1us tick
	tx_chan_cfg.mem_block_symbols = 64;			   // number of symbols the HW can buffer
	tx_chan_cfg.trans_queue_depth = 4;			   // number of pending transactions
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &s_rmt_tx_chan));
	ESP_ERROR_CHECK(rmt_enable(s_rmt_tx_chan));

	// Use copy encoder to send raw RMT symbols
	rmt_copy_encoder_config_t copy_cfg{};
	ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &s_rmt_copy_enc));
	LOG_TX("RMT TX channel initialized on GPIO %d", (int)output_GPIO);

	// Create TX queue
	g_tx_queue = xQueueCreate(TX_QUEUE_LEN, sizeof(tx_data_t));
	if (!g_tx_queue)
		return ESP_ERR_NO_MEM;

	// Create TX task
	BaseType_t ok = xTaskCreate(tx_task, "rz_tx_task", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);
	if (ok != pdPASS)
		return ESP_FAIL;
	LOG_TX("TX task created and started");
	return ESP_OK;
}

