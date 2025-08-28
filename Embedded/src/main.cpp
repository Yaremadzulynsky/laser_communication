#include "receiver.hpp" // RX state machine and initialization
#include "transmitter.hpp" // TX framing and initialization

#if ENABLE_RX
void consume_byte(uint8_t payload_byte)
{
	putchar((int)payload_byte);
	fflush(stdout);
}

#endif

extern "C" void app_main(void)
{

	// Initialize RX if enabled
#if ENABLE_RX
	ESP_ERROR_CHECK(laser_rx_init()); // expect 1-byte payload frames
#endif

// Initialize TX if enabled
#if ENABLE_TX
	ESP_ERROR_CHECK(laser_tx_init());

	// Send a test message continuously.

	// Test message to send
	const char test_message[] = "Laser Communication\n";
	static_assert((sizeof(test_message) - 1) <= MAX_PAYLOAD_LENGTH, "Test payload exceeds MAX_PAYLOAD_LENGTH");
	tx_data_t tx_data;

	// remove null terminator from char array
	memcpy(tx_data.data, test_message, sizeof(test_message) - 1);
	tx_data.len = sizeof(test_message) - 1;
	// loop forever
	for (;;)
	{
		// enqueue payload; TX task sends frame
		laser_tx_write(tx_data, portMAX_DELAY);
		// wait between sends
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
#endif
}
