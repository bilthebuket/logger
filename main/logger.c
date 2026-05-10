#include <stdio.h>
#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"

#define UART_BUFFER_SIZE 4096
#define EVENT_QUEUE_SIZE 10 // max number of events in the event queue
#define TX_PIN GPIO_NUM_1
#define RX_PIN GPIO_NUM_2

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

	char buf[UART_BUFFER_SIZE];
	while (1)
	{
		size_t length = uart_read_bytes(UART_NUM_2, (uint8_t*) &buf[0], UART_BUFFER_SIZE, pdMS_TO_TICKS(100));
		
		int start = 0;
		for (int i = 0; i < length; i++)
		{
			if (buf[i] == '\r' && buf[i + 1] == '\n')
			{
				nmea_s* msg = nmea_parse(&buf[start], i - start + 2, 0);

				if (msg != NULL)
				{
					switch (msg->type)
					{
						default:
						{
							printf("Other type\n");
							break;
						}
						case NMEA_GPGLL:
						{
							nmea_gpgll_s *gpgll = (nmea_gpgll_s*) msg;
							printf("Long: %d\n", gpgll->longitude.degrees);
							break;
						}
						
						case NMEA_GPGGA:
						{
							nmea_gpgga_s* gpgga = (nmea_gpgga_s*) msg;
							printf("Alt: %f %c\n", gpgga->altitude, gpgga->altitude_unit);
							break;
						}
					}

					free(msg);
				}

				start = i + 2;
			}
		}
	}
}
