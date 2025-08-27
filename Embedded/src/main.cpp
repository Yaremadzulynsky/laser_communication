#include "defines.hpp"
#include "esp_task_wdt.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

// ================== GPIO CONFIG ==================
static constexpr gpio_num_t LASER_GPIO = GPIO_NUM_13;
static constexpr gpio_num_t LASER_RX_GPIO = GPIO_NUM_26;

#if ENABLE_TX
// ============== SIMPLE RZ TX ================
// RZ convention:
// - bit 1: HIGH for first half-bit, then LOW for second half
// - bit 0: LOW for entire bit period

struct tx_data_t
{
	uint8_t data[128]; // 128 bytes
	size_t len;		   // length of the data
};

// ================== TX FreeRTOS QUEUE CONFIG ==================
static QueueHandle_t g_tx_queue = nullptr;
static constexpr size_t TX_QUEUE_LEN = 512; // adjust as needed

// ================== RMT CONFIG ==================
static rmt_channel_handle_t s_rmt_tx_chan = nullptr;
static rmt_encoder_handle_t s_rmt_copy_enc = nullptr;
static constexpr uint32_t RMT_RESOLUTION_HZ = 1000000; // 1 MHz -> 1 tick = 1 us

// ================== Forward declarations ==================
static void tx_task(void *arg);
static esp_err_t laser_tx_init(void);
static inline void rz_send_byte(uint8_t value);
static inline void rz_send_bit(int bit);
static inline void rz_send_frame(const uint8_t *payload, size_t len);
static void tx_task(void *arg);

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
	tx_chan_cfg.gpio_num = LASER_GPIO;
	tx_chan_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
	tx_chan_cfg.resolution_hz = RMT_RESOLUTION_HZ; // 1us tick
	tx_chan_cfg.mem_block_symbols = 64;			   // number of symbols the HW can buffer
	tx_chan_cfg.trans_queue_depth = 4;			   // number of pending transactions
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &s_rmt_tx_chan));
	ESP_ERROR_CHECK(rmt_enable(s_rmt_tx_chan));

	// Use copy encoder to send raw RMT symbols
	rmt_copy_encoder_config_t copy_cfg{};
	ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &s_rmt_copy_enc));
	LOG_TX("RMT TX channel initialized on GPIO %d", (int)LASER_GPIO);

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

static void tx_task(void *arg)
{
	(void)arg;
	tx_data_t tx_data;
	for (;;)
	{
		// Block until at least one byte is available
		if (xQueueReceive(g_tx_queue, &tx_data, portMAX_DELAY) == pdTRUE)
		{
			// Send each queued byte as its own frame
			rz_send_frame(&tx_data.data[0], 1);

			// Drain any immediately available bytes without blocking
			while (xQueueReceive(g_tx_queue, &tx_data, 0) == pdTRUE)
			{
				rz_send_frame(&tx_data.data[0], 1);
			}
		}
	}
}

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

	// send a start bit
	rz_send_bit(1);
	// Send payload
	for (size_t i = 0; i < len; ++i)
	{
		rz_send_byte(payload[i]);
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

	// LOG_TX("Sent bytes: %s", data);

	return ESP_OK;
}

#endif

#if ENABLE_RX
// ================== SIMPLE RZ RX ==================
/**
 * @brief RX state machine states
 */
typedef enum
{
	RX_SEARCH_PREAMBLE = 0,
	RX_SEARCH_START_OF_FRAME = 1,
	RX_COLLECT_PAYLOAD = 2,
	RX_SEARCH_STOP = 3
} rx_state_t;

// uint64 shift register to store 8 bit timestamps
static uint64_t g_rx_bit_times[8];
static uint8_t g_rx_bit_times_measured = 0;

/**
 * @brief RX state
 *
 * @param rx_gpio the GPIO number of the RX pin
 * @param last_rising_us the last rising edge time
 * @param have_prev_edge whether we have a previous edge
 * @param current_byte the current byte we are assembling
 * @param bit_pos the current bit position in the current byte we are assembling
 * @param state the current state
 * @param preamble_found_count the number of consecutive preamble bytes found
 * @param payload_bytes_expected the number of bytes expected in the payload we are collecting
 * @param payload_bytes_collected the number of bytes we have collected thus far
 */
typedef struct
{
	gpio_num_t rx_gpio;		// the GPIO number of the RX pin
	int64_t last_rising_us; // the last rising edge time

	// frequency and phase lock
	bool have_prev_edge;		// whether we have a previous edge
	int64_t locked_bit_time_us; // the time transmitter takes to send a bit

	// Bit/byte assembly (LSB-first)
	uint8_t current_byte; // the current byte we are assembling
	int bit_pos;		  // 0..7 // the current bit position in the current byte we are assembling

	// Framing
	rx_state_t state;			 // the current state
	int preamble_found_count;	 // the number of consecutive preamble bytes found
	int payload_bytes_expected;	 // the number of bytes expected in the payload we are collecting
	int payload_bytes_collected; // the number of bytes we have collected thus far
} laser_rx_t;

// ================== RX GLOBAL VARIABLES ==================
static laser_rx_t g_rx{};
static QueueHandle_t rx_edge_queue = nullptr;
static QueueHandle_t isr_log_queue = nullptr;

// ================== RX HELPER FUNCTIONS ==================

#include <stdarg.h>
#include <stdio.h>

// Choose a max message length for safety
#define ISR_LOG_BUF_SIZE 512
static bool lock_frequency(int64_t now_us);
static void log_in_isr(int64_t now_us, BaseType_t *awoken, const char *fmt, ...)
{
	char buf[ISR_LOG_BUF_SIZE];

	// Format the message
	va_list args;
	va_start(args, awoken);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len < 0)
	{
		return; // formatting error
	}

	if (len >= ISR_LOG_BUF_SIZE)
	{
		len = ISR_LOG_BUF_SIZE - 1; // truncate safely
	}

	// Push each char into the ISR queue
	for (int i = 0; i < len; i++)
	{
		xQueueSendFromISR(isr_log_queue, &buf[i], awoken);
	}
}

/**
 * @brief Reset the laser_rx_t bit/byte assembly values.
 */
static inline void rx_reset_assembly(void)
{
	g_rx.current_byte = 0;
	g_rx.bit_pos = 0;
	g_rx.have_prev_edge = false;
	g_rx.last_rising_us = 0;
	g_rx.preamble_found_count = 0;
	g_rx.payload_bytes_collected = 0;
}

/**
 * @brief Check if the byte is the preamble byte. If it is, increment the preamble found count.
 * If the preamble found count is greater than or equal to the preamble reps, set the state to
 * RX_SEARCH_START_OF_FRAME and reset the assembly.
 *
 */
static inline void rx_check_preamble()
{
	// check if current byte is the preamble byte
	if (g_rx.current_byte == PREAMBLE_BYTE)
	{
		//
		// shift the bit time window to the left by 1
		// for (int i = 0; i < 7; ++i)
		// {
		// 	g_rx_bit_times[i] = g_rx_bit_times[i + 1];
		// }
		// // store the bit time
		// g_rx_bit_times[7] = now_us;

		// check if there are any zeros in the bit time window
		// for (int i = 0; i < 7; ++i)
		// {
		// 	if (g_rx_bit_times[i] == 0)
		// 	{
		// 		return;
		// 	}
		// }

		// LOG_RX("Bit time window: %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld\n", g_rx_bit_times[0], g_rx_bit_times[1], g_rx_bit_times[2], g_rx_bit_times[3], g_rx_bit_times[4], g_rx_bit_times[5], g_rx_bit_times[6], g_rx_bit_times[7]);

		// find the min and max delta between adjacent bit times
		int64_t min_delta = INT64_MAX;
		int64_t max_delta = 0;
		for (int i = 0; i < 7; ++i)
		{
			int64_t delta = g_rx_bit_times[i + 1] - g_rx_bit_times[i];
			if (delta < min_delta)
			{
				min_delta = delta;
			}
			if (delta > max_delta)
			{
				max_delta = delta;
			}
		}

		// print the min and max delta
		// LOG_RX("Min delta: %lld, Max delta: %lld, Delta: %lld, Tolerance: %d\n", min_delta, max_delta, max_delta - min_delta, FREQUENCY_TOLERANCE_US);

		// print the frequency tolerance
		// LOG_RX("Frequency tolerance: %lld us", FREQUENCY_TOLERANCE_US);

		// check if the frequency is locked
		if (max_delta - min_delta < FREQUENCY_TOLERANCE_US)
		{
			// LOG_RX("Frequency locked, entering search start of frame state");
			g_rx.locked_bit_time_us = (max_delta + min_delta) / 2;
			LOG_RX("Bit time locked to: %lld us", g_rx.locked_bit_time_us);
		}
		else
		{
			return;
		}

		/*
		we may want more than one preamble byte to lock the timing so increment the preamble found count.
		when the preamble found count is greater than or equal to the preamble reps, set the state to
		RX_SEARCH_START_OF_FRAME and reset the assembly.
		*/
		g_rx.preamble_found_count++;

		// if we have recieved the required number of preamble bytes
		if (g_rx.preamble_found_count >= PREAMBLE_REPS)
		{
			// calculate the bit time
			// g_rx.locked_bit_time_us = (g_rx_bit_times[7] - g_rx_bit_times[0]) / 7;
			// log the bit time
			// LOG_RX("Bit time: %lld us", g_rx.locked_bit_time_us);

			// reset the assembly so we move to the next state
			rx_reset_assembly();
			// set the state to RX_SEARCH_START_OF_FRAME which is the next state.
			g_rx.state = RX_SEARCH_START_OF_FRAME;
			// log that we have locked the preamble and can search for the start of frame byte.
			LOG_RX("Preamble locked (%d x 0x%02X)", PREAMBLE_REPS, PREAMBLE_BYTE);
		}
	}
}

/**
 * @brief Check if the byte is the start of frame byte. If it is, set the state to
 * RX_COLLECT_PAYLOAD and reset the assembly.
 *
 */
static inline void rx_check_start_of_frame()
{
	// check if current byte is the start of frame byte
	if (g_rx.current_byte == START_OF_FRAME_BYTE)
	{
		// reset the assembly so we move to the next state
		rx_reset_assembly();
		// set the state to RX_COLLECT_PAYLOAD which is the next state.
		g_rx.state = RX_COLLECT_PAYLOAD;
		// log that we have locked the start of frame byte and can collect the payload.
		LOG_RX("Start of frame byte found, entering collect payload state");
	}
	else
	{
		// reset the assembly so we move to the next state
		rx_reset_assembly();
		// set the state to RX_SEARCH_PREAMBLE which is the next state.
		g_rx.state = RX_SEARCH_PREAMBLE;
		LOG_RX("Start of frame byte not found, continuing to search preamble");
	}
}

/**
 * @brief Collect the payload byte. If the payload bytes collected is greater than or equal to the payload bytes expected, set the state to
 * RX_SEARCH_STOP and reset the assembly.
 *
 */
static inline void rx_collect_payload()
{
	LOG_RX("Collecting payload byte: 0x%02X", g_rx.current_byte);

	printf("%c", g_rx.current_byte);
	fflush(stdout);
	// increment the payload bytes collected count
	g_rx.payload_bytes_collected++;

	// if the payload bytes collected is greater than or equal to the payload bytes expected
	if (g_rx.payload_bytes_collected >= g_rx.payload_bytes_expected)
	{
		// print the byte we received
		// printf("%c", g_rx.current_byte);
		// fflush(stdout);

		// reset the assembly so we move to the next state
		rx_reset_assembly();
		// set the state to RX_SEARCH_STOP which is the next state.
		g_rx.state = RX_SEARCH_STOP;

		// log that we have collected the payload and can search for the stop byte.
		LOG_RX("Payload collected, entering search stop state");
	}
}

/**
 * @brief Check if the byte is the stop byte. If it is, set the state to
 * RX_SEARCH_PREAMBLE and reset the assembly.
 *
 */
static inline void rx_check_stop()
{
	// check if current byte is the stop byte
	if (g_rx.current_byte == END_OF_FRAME_BYTE)
	{
		// reset the assembly so we move to the next state
		rx_reset_assembly();
		// set the state to RX_SEARCH_PREAMBLE which is the next state (the intial state).
		g_rx.state = RX_SEARCH_PREAMBLE;
		// log that we have collected the payload and can search for the stop byte.
		LOG_RX("Stop byte found, resetting to search preamble");
	}
	else
	{
		// reset the assembly so we move to the next state
		rx_reset_assembly();
		// set the state to RX_SEARCH_PREAMBLE which is the next state.
		g_rx.state = RX_SEARCH_PREAMBLE;
		LOG_RX("Stop byte not found, resetting to search preamble");
	}
}

/**
 * @brief Push the completed byte to the RX state machine.
 *
 * @note This function is called by rx_push_bit when we have collected a complete byte.
 */
static inline void rx_push_completed_byte()
{
	// useful for debugging to print the byte we received and the state we are in.
	// LOG_RX("RX byte: 0x%02X state: %d", g_rx.current_byte, g_rx.state);

	// print the current state and byte
	LOG_RX("RX state: %d, byte: 0x%02X/%c", g_rx.state, g_rx.current_byte, g_rx.current_byte);
	// check the state and call the appropriate function to handle the state.
	switch (g_rx.state)
	{
	case RX_SEARCH_PREAMBLE: // we are searching for the preamble byte
		rx_check_preamble();
		break;
	case RX_SEARCH_START_OF_FRAME: // we are searching for the start of frame byte
		rx_check_start_of_frame();
		break;
	case RX_COLLECT_PAYLOAD: // we are collecting the payload bytes
		rx_collect_payload();
		break;
	case RX_SEARCH_STOP: // we are searching for the stop byte
		rx_check_stop();
		// reset frequency lock
		g_rx.locked_bit_time_us = UINT64_MAX;

		break;
	default:
		break;
	}
}

/**
 * @brief Push a bit into our software shift register (g_rx.current_byte).
 *
 * @note This function is called by rx_clock_bits_during_delta_us.
 */
static inline void rx_push_bit(int bit)
{
	// push the bit into our software shift register (g_rx.current_byte).
	if (bit)
	{
		g_rx.current_byte |= (uint8_t)(1u << g_rx.bit_pos);
	}
	// increment the bit position
	g_rx.bit_pos++;

	// printf(bit ? "1" : "0");
	// fflush(stdout);

	// if we have collected 8 bits, push the completed byte to the RX state machine.
	if (g_rx.bit_pos >= 8)
	{
		// printf("\n");
		// printf("bit_pos: %d\n", g_rx.bit_pos);
		// fflush(stdout);
		// log the byte we received
		LOG_RX("RX byte: 0x%02X", (unsigned)g_rx.current_byte);
		// push the completed byte to the RX state machine.
		rx_push_completed_byte();
		g_rx.bit_pos = 0;
	}
}

/**
 * @brief
 *
 * @param delta_us
 */
static inline void rx_clock_bits_during_delta_us(int64_t delta_us, uint64_t time_stamp_us)
{

	// if (g_rx.state == RX_SEARCH_PREAMBLE)
	// {
	// 	if (!lock_frequency(time_stamp_us))
	// 	{
	// 		LOG_RX("Frequency not locked, returning");
	// 		return;
	// 	}
	// }

	// convert the bit time to microseconds
	const int64_t BIT_US = g_rx.locked_bit_time_us;
	// const int64_t BIT_US = (int64_t)BIT_MS * 1000;

	// if the delta is less than or equal to 0 something went wrong so return.
	if (delta_us <= 0)
		return;

	// Round to nearest integer number of bits since our last rising edge.
	int64_t gap_bits64 = (delta_us + (BIT_US / 2)) / BIT_US;

	// if the gap is less than 1, set it to 1 as at least one bit must have elapsed.
	if (gap_bits64 < 1)
		gap_bits64 = 1;

	// if the gap is greater than 64 clamp it to 64
	if (gap_bits64 > 64)
		gap_bits64 = 64; // clamp to avoid runaway

	// convert the gap to an integer
	int gap_bits = (int)gap_bits64;

	/*
	push the gap bits. Notice we push the gap bits - 1 as the last bit is the 1 bit
	we got from the last rising edge.
	*/
	for (int i = 0; i < gap_bits - 1; ++i)
		rx_push_bit(0);

	// push a 1 bit as we have elapsed the gap bits.
	rx_push_bit(1);
}

/**
 * @brief This task waits for an edge to be pushed into the queue and then shifts the bits
 * into the software shift register (g_rx.current_byte).
 *
 * @param arg  pointer to the argument (unused)
 */
static void rx_task(void *arg)
{
	// void the arg to keep the compiler happy.
	(void)arg;

	// set the initial state to RX_SEARCH_PREAMBLE.
	g_rx.state = RX_SEARCH_PREAMBLE;

	// loop forever
	for (;;)
	{
		// create a variable used to store the time of the last rising edge.
		int64_t now_us = 0;

		/*
		Wait for an edge to be pushed into the queue. note that since we set the third argument
		to portMAX_DELAY, the task will block until an edge is pushed into the queue. Edges are pushed
		into the queue by the ISR (rx_gpio_isr) which is triggered by the rising edge of the laser RX GPIO.
		This effectively tells the task to idle until an edge has been detected and therefore a 1 bit has
		been received.

		*/
		if (xQueueReceive(rx_edge_queue, &now_us, portMAX_DELAY))
		{
			for (int i = 0; i < 7; ++i)
			{
				g_rx_bit_times[i] = g_rx_bit_times[i + 1];
			}
			// store the bit time
			g_rx_bit_times[7] = now_us;
			// print the current time
			// LOG_RX("Current time: %lld", now_us);

			// print the array of bit times
			//  LOG_RX("Bit times: %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld\n", g_rx_bit_times[0], g_rx_bit_times[1], g_rx_bit_times[2], g_rx_bit_times[3], g_rx_bit_times[4], g_rx_bit_times[5], g_rx_bit_times[6], g_rx_bit_times[7]);

			// if (g_rx.state == RX_SEARCH_PREAMBLE)
			// {
			// 	// print searching for preamble
			// 	LOG_RX("Searching for preamble");
			// 	if (lock_frequency(now_us))
			// 	{
			// 	        // DO NOT push bits here
			// 			rx_reset_assembly();                  // clear byte & bit_pos
			// 			g_rx.have_prev_edge = false;          // eat the very next edge to re-phase
			// 			g_rx.last_rising_us = now_us;         // mark time
			// 			g_rx.state = RX_SEARCH_START_OF_FRAME;
			// 			LOG_RX("Preamble locked, searching SOF");
			// 	}
			// }
			// else
			// {
			// 	// print elsewhere
			// LOG_RX("RX state: %d", g_rx.state);
			// LOG_RX("RX state: %d", g_rx.state);
			/*
			If we havent had an edge yet, this means the initial rising edge has been detected.
			This rising edge is NOT part of the byte, but is there to lock the timing in case
			the byte has leading zeros. This bit essentailly locks the phase of the RX task to the
			transmitter.
			*/
			if (!g_rx.have_prev_edge)
			{
				// set the have_prev_edge flag to true
				g_rx.have_prev_edge = true;
				g_rx.last_rising_us = now_us;
				// continue to the next iteration of the loop.
				continue;
			}

			// calculate the delta between the current and last rising edge.
			int64_t delta = now_us - g_rx.last_rising_us;

			// update the last rising edge time.
			g_rx.last_rising_us = now_us;

			// // shift the bit times
			// for (int i = 0; i < 7; ++i)
			// {
			// 	g_rx_bit_times[i] = g_rx_bit_times[i + 1];
			// }
			// // store the bit time
			// g_rx_bit_times[7] = now_us;

			// process the
			rx_clock_bits_during_delta_us(delta, now_us);
		}
	}
	// }
}

static void log_task(void *arg)
{
	(void)arg;

	for (;;)
	{
		char byte = 0;
		if (xQueueReceive(isr_log_queue, &byte, portMAX_DELAY))
		{
			printf("%c", byte);
			fflush(stdout);
		}
	}
}

static bool lock_frequency(int64_t now_us)
{

	// shift the bit time window to the left by 1
	for (int i = 0; i < 7; ++i)
	{
		g_rx_bit_times[i] = g_rx_bit_times[i + 1];
	}
	// store the bit time
	g_rx_bit_times[7] = now_us;

	// check if there are any zeros in the bit time window
	for (int i = 0; i < 7; ++i)
	{
		if (g_rx_bit_times[i] == 0)
		{
			return false;
		}
	}

	// LOG_RX("Bit time window: %lld, %lld, %lld, %lld, %lld, %lld, %lld, %lld\n", g_rx_bit_times[0], g_rx_bit_times[1], g_rx_bit_times[2], g_rx_bit_times[3], g_rx_bit_times[4], g_rx_bit_times[5], g_rx_bit_times[6], g_rx_bit_times[7]);

	// find the min and max delta between adjacent bit times
	int64_t min_delta = INT64_MAX;
	int64_t max_delta = 0;
	for (int i = 0; i < 7; ++i)
	{
		int64_t delta = g_rx_bit_times[i + 1] - g_rx_bit_times[i];
		if (delta < min_delta)
		{
			min_delta = delta;
		}
		if (delta > max_delta)
		{
			max_delta = delta;
		}
	}

	// print the min and max delta
	// LOG_RX("Min delta: %lld, Max delta: %lld, Delta: %lld, Tolerance: %d\n", min_delta, max_delta, max_delta - min_delta, FREQUENCY_TOLERANCE_US);

	// print the frequency tolerance
	// LOG_RX("Frequency tolerance: %lld us", FREQUENCY_TOLERANCE_US);

	// check if the frequency is locked
	if (max_delta - min_delta < FREQUENCY_TOLERANCE_US)
	{
		// LOG_RX("Frequency locked, entering search start of frame state");
		g_rx.locked_bit_time_us = (max_delta + min_delta) / 2;
		LOG_RX("Bit time locked to: %lld us", g_rx.locked_bit_time_us);
		return true;
	}
	return false;
}

/**
 * @brief ISR for the laser RX GPIO. This ISR is triggered by the rising edge of the laser RX GPIO.
 *
 *
 * @param arg pointer to the argument (unused)
 */
static void IRAM_ATTR rx_gpio_isr(void *arg)
{
	// void the arg to keep the compiler happy.
	(void)arg;

	// get the current time in microseconds.
	int64_t now_us = esp_timer_get_time();

	// create a variable to store the time of the last ISR.
	static int64_t s_last_isr_us = 0;

	// calculate the time since the last ISR.
	int64_t since_last = now_us - s_last_isr_us;

	// if the time since the last ISR is less than the debounce time, return.
	if (since_last >= 0 && since_last < RX_DEBOUNCE_US)
	{
		// printf("debounce: %lld\n", since_last);
		// fflush(stdout);
		return;
	}

	// update the last ISR time.
	s_last_isr_us = now_us;
	BaseType_t awoken = pdFALSE;

	// if the edge queue is not null, push the current time into the queue.
	if (rx_edge_queue)
	{
		xQueueSendFromISR(rx_edge_queue, &now_us, &awoken);
	}

	// If a higher-priority task was woken by xQueueSendFromISR, yield to it.
	if (awoken == pdTRUE)
	{
		portYIELD_FROM_ISR();
	}
}

/**
 * @brief Initialize the laser RX GPIO and create the RX task.
 *
 * @param expected_payload_bytes the number of bytes expected in a payload
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t laser_rx_init(int expected_payload_bytes)
{
	// configure the RX GPIO as input with rising-edge interrupt
	gpio_config_t gc{};
	gc.pin_bit_mask = 1ULL << LASER_RX_GPIO;
	gc.mode = GPIO_MODE_INPUT;
	gc.pull_up_en = GPIO_PULLUP_DISABLE;
	gc.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gc.intr_type = GPIO_INTR_POSEDGE;

	// configure the RX GPIO
	ESP_ERROR_CHECK(gpio_config(&gc));

	// set the RX GPIO number
	g_rx.rx_gpio = LASER_RX_GPIO;

	// set the number of bytes expected in a payload
	g_rx.payload_bytes_expected = expected_payload_bytes;

	// we do not have a phase lock yet.
	g_rx.have_prev_edge = false;

	g_rx.locked_bit_time_us = UINT64_MAX;

	// create the edge queue and install the ISR service
	rx_edge_queue = xQueueCreate(64, sizeof(int64_t));

	// if the edge queue is not created, return an error.
	if (!rx_edge_queue)
		return ESP_ERR_NO_MEM;

	// install the ISR service
	esp_err_t isr_ok = gpio_install_isr_service(0);

	// if the ISR service is not installed, return an error.
	if (isr_ok != ESP_OK && isr_ok != ESP_ERR_INVALID_STATE)
	{
		return isr_ok;
	}

	// add the ISR service to the RX GPIO.
	ESP_ERROR_CHECK(gpio_isr_handler_add(LASER_RX_GPIO, rx_gpio_isr, nullptr));

	// enable the interrupt on the RX GPIO.
	ESP_ERROR_CHECK(gpio_intr_enable(LASER_RX_GPIO));

	// launch the RX task (slightly above idle priority)
	BaseType_t ok = xTaskCreate(rx_task, "rz_rx_task", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

	// if the RX task is not created, return an error.
	if (ok != pdPASS)
		return ESP_FAIL;

	// return success.
	return ESP_OK;
}
#endif

static void log_init(void)
{
	// create the isr log queue
	isr_log_queue = xQueueCreate(64, sizeof(char));

	// if the isr log queue is not created, return an error.
	if (!isr_log_queue)
		return;

	// launch the log task (slightly above idle priority)
	BaseType_t ok = xTaskCreate(log_task, "log_task", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

	// if the log task is not created, return an error.
	if (ok != pdPASS)
		return;
}

extern "C" void app_main(void)
{
	// initialize the log task
	log_init();

	// if RX is enabled, initialize the RX GPIO and create the RX task.
#if ENABLE_RX
	ESP_ERROR_CHECK(laser_rx_init(1)); // expect 1-byte payload frames
#endif

// if TX is enabled, initialize the TX GPIO and create the TX task.
#if ENABLE_TX
	ESP_ERROR_CHECK(laser_tx_init());
#endif

// if TX is enabled, send a test message continuously.
#if ENABLE_TX
	// create a test message to send.
	// const uint8_t hello[] = {(uint8_t)'H', (uint8_t)'e', (uint8_t)'l', (uint8_t)'l', (uint8_t)'o', (uint8_t)' ', (uint8_t)'W', (uint8_t)'o', (uint8_t)'r', (uint8_t)'l', (uint8_t)'d', (uint8_t)'!', (uint8_t)'\n'};
	const uint8_t hello[] = {(uint8_t)'H', (uint8_t)'e', (uint8_t)'\n'};
	// const uint8_t hello[] = {0xAA};
	tx_data_t tx_data;
	memcpy(tx_data.data, hello, sizeof(hello));
	tx_data.len = sizeof(hello);
	// loop forever
	for (;;)
	{
		// send the test message
		// for (size_t i = 0; i < sizeof(hello); ++i)
		// {
			// tx_data.data[0] = hello[i];
			// enqueue one byte at a time; TX task will send frames
			laser_tx_write(tx_data, portMAX_DELAY);
		// }
		// wait for 2.5 seconds
		vTaskDelay(pdMS_TO_TICKS(7500));
	}
#else
	// wait forever
	for (;;)
	{
		vTaskDelay(pdMS_TO_TICKS(2500));
	}
#endif
}
