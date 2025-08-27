#include "common.hpp" // shared includes (ESP-IDF, FreeRTOS) and project defines

/**
 * @brief RX state machine states
 */
typedef enum
{
	PREAMBLE = 0, // we are searching for the preamble byte
	SOF = 1, // we are searching for the start of frame byte
	PAYLOAD_LEN = 2, // we are collecting the length of the payload
	PAYLOAD = 3, // we are collecting the payload bytes
	_EOF = 4 // we are searching for the stop byte
} state_t;

typedef struct
{
	uint8_t byte;		 // the current byte we are assembling
	int bit_pos;		 // 0..7 // the current bit position in the current byte we are assembling
	int bytes_expected;	 // the number of bytes expected in the payload we are collecting
	int bytes_collected; // the number of bytes we have collected thus far
} assembly_t;

typedef struct
{
	int64_t bit_time_us; // the time transmitter takes to send a bit
	int preamble_count;	 // the number of consecutive preamble bytes found
} frequency_lock_t;

// ================== MODULE STATE ==================
static constexpr gpio_num_t input_GPIO = GPIO_NUM_26; // RX pin
static uint64_t rising_edge_times[PREAMBLE_REPS * 8]; // shift register of rising edge timestamps
static QueueHandle_t rx_edge_queue = nullptr; // queue of edge timestamps

bool have_prev_edge; // whether we have a previous edge
int64_t last_rising_us; // the last rising edge time
state_t state; // current RX state
assembly_t assembly; // current byte/payload assembly
frequency_lock_t lock; // current frequency lock

// ================== RX HELPER FUNCTIONS ==================

void consume_byte(uint8_t payload_byte);

/**
 * @brief Reset all RX assembly and transient timing fields.
 */
static void rx_reset_assembly(void)
{
	assembly.byte = 0;
	assembly.bit_pos = 0;
	have_prev_edge = false;
	last_rising_us = 0;
	lock.preamble_count = 0;
	assembly.bytes_collected = 0;
}

/**
 * @brief Reset only the current byte-building fields (keep payload counters).
 */
static inline void rx_reset_byte_only(void)
{
	assembly.byte = 0;
	assembly.bit_pos = 0;
	have_prev_edge = false;
	last_rising_us = 0;
}

/**
 * @brief Attempt to lock the bit time based on recent rising-edge deltas.
 * Resets byte/timing fields but preserves preamble_count; returns true if locked.
 */
static bool rx_check_frequency_lock()
{
	// find the min and max delta between adjacent bit times
	int64_t min_delta = INT64_MAX;
	int64_t max_delta = 0;
	for (int i = 0; i < PREAMBLE_REPS * 8 - 1; ++i)
	{
		int64_t delta = rising_edge_times[i + 1] - rising_edge_times[i];
		if (delta < min_delta)
		{
			min_delta = delta;
		}
		if (delta > max_delta)
		{
			max_delta = delta;
		}
	}

	// reset byte/timing but keep preamble_count to continue lock detection
	rx_reset_byte_only();
	assembly.bytes_collected = 0;

	// check if the frequency is locked
	if (max_delta - min_delta < FREQUENCY_TOLERANCE_US)
	{
		lock.bit_time_us = (max_delta + min_delta) / 2;
		LOG_RX("Bit time locked to: %lld us", lock.bit_time_us);
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * @brief Check if the byte is the preamble byte. If it is, increment the preamble found count.
 * If the preamble found count is greater than or equal to the preamble reps, set the state to
 * SOF and reset the assembly.
 *
 */
inline static void rx_handle_preamble()
{
	// ignore non-preamble bytes
	if (assembly.byte != PREAMBLE_BYTE)
		return;

	lock.preamble_count++;

	if (!rx_check_frequency_lock())
		return;

	// if we have received the required number of preamble bytes
	if (lock.preamble_count < PREAMBLE_REPS)
		return;

	// reset the assembly so we move to the next state
	rx_reset_assembly();
	// set the state to SOF which is the next state.
	state = SOF;
	// log that we have locked the preamble and can search for the start of frame byte.
	LOG_RX("Preamble locked (%d x 0x%02X)", PREAMBLE_REPS, PREAMBLE_BYTE);
}

/**
 * @brief Check if the byte is the start of frame byte.
 * If found: set state to PAYLOAD_LEN; otherwise: PREAMBLE. Resets assembly.
 */
inline static void rx_handle_sof()
{
	// check if current byte is the start of frame byte
	const bool sof_found = (assembly.byte == START_OF_FRAME_BYTE);

	state = sof_found ? PAYLOAD_LEN : PREAMBLE;

	if (sof_found)
	{
		LOG_RX("Start of frame byte found, entering collect payload state");
	}
	else
	{
		lock.bit_time_us = UINT64_MAX;
	}

	// Reset assembly after processing SOF result
	rx_reset_assembly();
}

/**
 * @brief Collect one payload byte. When collected count reaches expected length,
 * set state to _EOF and reset the assembly.
 */
inline static void rx_handle_payload()
{
	assembly.byte -= 128;
	// LOG_RX("Collecting payload byte: 0x%02X", assembly.byte);

	if (assembly.byte > 128)
	{
		printf("\nInvalid payload byte detected, searching for preamble\n");
		fflush(stdout);
		LOG_RX("Payload byte is greater than 128 error detected, resetting assembly");
		state = PREAMBLE;
		lock.bit_time_us = UINT64_MAX;
		rx_reset_assembly();
		return;
	}
	consume_byte(assembly.byte);

	// increment the payload bytes collected count
	assembly.bytes_collected++;

	// continue collecting until expected length is reached
	if (assembly.bytes_collected < assembly.bytes_expected)
	{
		rx_reset_byte_only();
		lock.preamble_count = 0;
		return;
	}

	// reached expected length
	state = _EOF;
	LOG_RX("Payload collected, entering search stop state");
	rx_reset_assembly();
}

inline static void rx_handle_length()
{
	const uint8_t length = assembly.byte - 128;
	assembly.bytes_expected = length;
	LOG_RX("Length parsed: %u bytes", (unsigned)length);
	state = PAYLOAD;
	rx_reset_byte_only();
	fflush(stdout);
}

/**
 * @brief Check if the byte is the stop byte. If it is, set the state to
 * PREAMBLE and reset the assembly.
 *
 */
inline static void rx_handle_eof()
{
	// check if current byte is the stop byte
	const bool stop_found = (assembly.byte == END_OF_FRAME_BYTE);
	state = PREAMBLE;
	if (stop_found)
	{
		LOG_RX("Stop byte found, resetting to search preamble");
	}
	else
	{
		lock.bit_time_us = UINT64_MAX;
		LOG_RX("Stop byte not found, resetting to search preamble");
	}
	rx_reset_assembly();
}


/**
 * @brief Dispatch a completed byte to the appropriate state handler.
 * Called after 8 bits are assembled; handlers may advance state and reset assembly.
 */
static void rx_push_completed_byte()
{
	// check the state and call the appropriate function to handle the state.
	switch (state)
	{
	case PREAMBLE: // we are searching for the preamble byte
		rx_handle_preamble();
		break;
	case SOF: // we are searching for the start of frame byte
		rx_handle_sof();
		break;
	case PAYLOAD_LEN: // we are collecting the length of the payload
		rx_handle_length(); // only byte reset here
		break;
	case PAYLOAD: // we are collecting the payload bytes
		rx_handle_payload(); // resets byte-only until done; full reset when done
		break;
	case _EOF: // we are searching for the stop byte
		rx_handle_eof();
		// reset frequency lock
		lock.bit_time_us = UINT64_MAX;
		break;
	default:
		break;
	}
}

/**
 * @brief Push a bit into the software shift register (assembly.byte).
 * Called by rx_clock_bits_during_delta_us when timing says a bit elapsed.
 */
static void rx_push_bit(int bit)
{
	// push the bit into our software shift register (assembly.byte).
	if (bit)
	{
		assembly.byte |= (uint8_t)(1u << assembly.bit_pos);
	}
	// increment the bit position
	assembly.bit_pos++;

	// if we have collected 8 bits, push the completed byte to the RX state machine.
	if (assembly.bit_pos >= 8)
	{
		// push the completed byte to the RX state machine.
		rx_push_completed_byte();
		assembly.bit_pos = 0;
	}
}

/**
 * @brief Clock bits based on time delta since last rising edge.
 * Rounds to nearest number of bit-times and pushes (gap_bits-1) zeros and then a one.
 */
static void rx_clock_bits_during_delta_us(int64_t delta_us, uint64_t time_stamp_us)
{

	// convert the bit time to microseconds
	const int64_t BIT_US = lock.bit_time_us;
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
 * @brief RX task: wait for rising edges, compute elapsed bit-times, and feed bits.
 */
static void rx_task(void *arg)
{
	// void the arg to keep the compiler happy.
	(void)arg;

	// set the initial state to PREAMBLE.
	state = PREAMBLE;

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
			for (int i = 0; i < PREAMBLE_REPS * 8 - 1; ++i)
			{
				rising_edge_times[i] = rising_edge_times[i + 1];
			}
			// store the bit time
			rising_edge_times[PREAMBLE_REPS * 8 - 1] = now_us;

			/*
			If we havent had an edge yet, this means the initial rising edge has been detected.
			This rising edge is NOT part of the byte, but is there to lock the timing in case
			the byte has leading zeros. This bit essentailly locks the phase of the RX task to the
			transmitter.
			*/
			if (!have_prev_edge)
			{
				// set the have_prev_edge flag to true
				have_prev_edge = true;
				last_rising_us = now_us;
				// continue to the next iteration of the loop.
				continue;
			}

			// calculate the delta between the current and last rising edge.
			int64_t delta = now_us - last_rising_us;

			// update the last rising edge time.
			last_rising_us = now_us;

			// process the
			rx_clock_bits_during_delta_us(delta, now_us);
		}
	}
}

/**
 * @brief ISR for laser RX GPIO, triggered on rising edge. Queues timestamp to task.
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
 * @return esp_err_t ESP_OK if successful, ESP_FAIL otherwise
 */
static esp_err_t laser_rx_init()
{
	// configure the RX GPIO as input with rising-edge interrupt
	gpio_config_t gc{};
	gc.pin_bit_mask = 1ULL << input_GPIO;
	gc.mode = GPIO_MODE_INPUT;
	gc.pull_up_en = GPIO_PULLUP_DISABLE;
	gc.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gc.intr_type = GPIO_INTR_POSEDGE;

	// configure the RX GPIO
	ESP_ERROR_CHECK(gpio_config(&gc));

	// we do not have a phase lock yet.
	have_prev_edge = false;

	lock.bit_time_us = UINT64_MAX;

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
	ESP_ERROR_CHECK(gpio_isr_handler_add(input_GPIO, rx_gpio_isr, nullptr));

	// enable the interrupt on the RX GPIO.
	ESP_ERROR_CHECK(gpio_intr_enable(input_GPIO));

	// launch the RX task (slightly above idle priority)
	BaseType_t ok = xTaskCreate(rx_task, "rz_rx_task", 4096, nullptr, tskIDLE_PRIORITY + 1, nullptr);

	// if the RX task is not created, return an error.
	if (ok != pdPASS)
		return ESP_FAIL;

	// return success.
	return ESP_OK;
}
