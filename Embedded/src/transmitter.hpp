#include "common.hpp"


// NOTE: A 0-128 in memory is actually a 128-256 when sent via the laser as its
// sending values < 128 is not a reliable way to send data. The RX side recovers the 0-128 range.
struct tx_data_t
{
	uint8_t data[MAX_PAYLOAD_LENGTH]; // 128 bytes
	size_t len;						  // length of the data
};

// ================== TX FreeRTOS QUEUE CONFIG ==================
static QueueHandle_t g_tx_queue = nullptr;
static constexpr size_t TX_QUEUE_LEN = 512; // adjust as needed

// ================== RMT CONFIG ==================
static rmt_channel_handle_t s_rmt_tx_chan = nullptr;
static rmt_encoder_handle_t s_rmt_copy_enc = nullptr;

static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000; // 1 MHz -> 1 tick = 1 us


// ================== RZ HELPER FUNCTIONS ==================
/**
 * @brief Send a bit via RZ encoding.
 *
 * @note RZ convention:
 * - bit 1: HIGH for first half-bit, then LOW for second half
 * - bit 0: LOW for entire bit period
 *
 * @param bit 0 or 1
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
 * @brief Send a byte via RZ encoding.
 *
 * @param value 8-bit value to send
 */
static inline void rz_send_byte(uint8_t value)
{
	for (int i = 0; i < 8; ++i)
	{
		int bit = (value >> i) & 1;
		rz_send_bit(bit);
	}
}

uint8_t bytes_to_send = UINT8_MAX;
/**
 * @brief Send a frame via RZ encoding.
 *
 * @note Since we can only detect changes (rising edges) in the light signal (essentially
 * its derivative) we must always send a '1' bit first. The time until the next rising
 * edge tells us how many leading zeros were present before the next '1'.
 * This way, the receiver can reconstruct the original bit sequence,
 * including any leading zeros.
 *
 * @param payload pointer to the payload to send
 * @param len length of the payload to send
 */
static void rz_send_frame(const uint8_t *payload, size_t len)
{

	// Send preamble (PREAMBLE_REPS times)
	for (int i = 0; i < PREAMBLE_REPS; ++i)
	{
		// send a start bit (1) no need to send it as we are sending the preamble byte
		// rz_send_bit(1);
		rz_send_bit(1);
		rz_send_byte(PREAMBLE_BYTE);
	}

	// Send start of frame byte (START_OF_FRAME_BYTE)
	rz_send_bit(1);
	rz_send_byte(START_OF_FRAME_BYTE);

	// Send length of payload
	// rz_send_bit(1);
	// turn len into a byte
	// min reliable uint8_t is 128 just sub on the other side
	uint8_t bytes_to_send = (uint8_t)len + 128;

	// printf("bytes_to_send: %d\n", bytes_to_send);
	// fflush(stdout);
	rz_send_bit(1);
	// bytes_to_send -= 10;
	rz_send_byte(bytes_to_send);

	// send a start bit
	// rz_send_bit(1);
	// Send payload
	// printf("len: %d\n", len);
	// printf("payload: ");
	// for (size_t i = 0; i < len; ++i)
	// {
	// 	printf("%02X ", payload[i]);
	// }
	// printf("\n");
	// fflush(stdout);
	for (size_t i = 0; i < len; ++i)
	{
		rz_send_bit(1);
		rz_send_byte(payload[i] + 128);
	}

	// send a start bit
	rz_send_bit(1);
	// send a stop bit
	rz_send_byte(END_OF_FRAME_BYTE);
}

// ================== TX API ==================
/**
 * @brief Enqueue bytes to be sent by TX task as one-byte frames
 *
 * @param data pointer to the data to send
 * @param len length of the data to send
 * @param wait_ticks maximum time to wait for the queue to be available
 * @return esp_err_t ESP_OK if successful, ESP_ERR_INVALID_STATE if the queue is not initialized, ESP_ERR_TIMEOUT if the queue is full
 */
static inline esp_err_t laser_tx_write(tx_data_t tx_data, TickType_t wait_ticks)
{
	// Check if the TX queue is initialized
	if (!g_tx_queue)
		return ESP_ERR_INVALID_STATE;

	// Enqueue the bytes
	// for (size_t i = 0; i < len; ++i)
	// {
	// Send the byte to the TX queue
	if (xQueueSend(g_tx_queue, &tx_data, wait_ticks) != pdTRUE)
	{
		return ESP_ERR_TIMEOUT;
	}
	// }

	return ESP_OK;
}