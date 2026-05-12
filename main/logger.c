#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"

#include "lib.h"

#define UART_BUFFER_SIZE 4096
#define EVENT_QUEUE_SIZE 10 // max number of events in the event queue
#define TX_PIN GPIO_NUM_43
#define RX_PIN GPIO_NUM_44

#define METERS_TO_FEET_CONVERSION_FACTOR 3.28084

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

	float* average_speed = calloc(1, sizeof(float));
	float* max_speed = calloc(1, sizeof(float));
	int num_data_points = 0;

	time_t last_time; 
	float last_lat = 0.0;
	float last_long = 0.0;
	float last_alt = 0.0;

	void* values_to_display[2];
	values_to_display[MAX_SPEED] = max_speed;
	values_to_display[AVERAGE_SPEED] = average_speed;
	pthread_t thread;
	pthread_create(&thread, NULL, &thread_func, values_to_display);

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
						case NMEA_GPGGA:
						{
							nmea_gpgga_s* gpgga = (nmea_gpgga_s*) msg;
							float current_lat = sexagesimal_to_radians(gpgga->latitude.degrees, gpgga->latitude.minutes);
							float current_long = sexagesimal_to_radians(gpgga->longitude.degrees, gpgga->latitude.minutes);
							time_t current_time = mktime(&(gpgga->time));
							float current_alt = gpgga->altitude * METERS_TO_FEET_CONVERSION_FACTOR;

							if (num_data_points > 0)
							{
								float ground_distance = calculate_distance(last_lat, last_long, current_lat, current_long);
								float distance = pow(pow(ground_distance, 2) + pow(current_alt - last_alt, 2), .5);
								float seconds_passed = difftime(current_time, last_time);
								float speed = distance / seconds_passed;
								
								if (speed > *max_speed)
								{
									*max_speed = speed;
								}

								float new_average_speed = *average_speed * (num_data_points - 1) / num_data_points + speed / num_data_points;
								*average_speed = new_average_speed;
							}
							
							last_time = current_time;
							last_lat = current_lat;
							last_long = current_long;
							last_alt = current_alt;
							num_data_points++;
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
