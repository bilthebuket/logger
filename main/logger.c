#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"

#include "lib.h"

#define EVENT_QUEUE_SIZE 10 // max number of events in the event queue
#define TX_PIN GPIO_NUM_43
#define RX_PIN GPIO_NUM_44

#define UART_BUFFER_SIZE 4096

void app_main(void)
{
	QueueHandle_t uart_queue;
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, UART_BUFFER_SIZE, UART_BUFFER_SIZE, EVENT_QUEUE_SIZE, &uart_queue, 0));
	
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));

	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	process_data_from_gps_sensor(&uart_read_bytes);
}
