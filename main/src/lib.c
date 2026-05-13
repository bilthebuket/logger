#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>

#include "lib.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"

#include "nmea.h"
#include "gpgll.h"
#include "gpgga.h"

#define PI 3.141592653
#define EARTH_RADIUS_FEET 20903520

#define MAX_SPEED_LOW_DIGIT GPIO_NUM_1
#define MAX_SPEED_HIGH_DIGIT GPIO_NUM_2
#define AVERAGE_SPEED_LOW_DIGIT GPIO_NUM_3
#define AVERAGE_SPEED_HIGH_DIGIT GPIO_NUM_4

#define TOP_SEGMENT GPIO_NUM_5
#define UPPER_LEFT_SEGMENT GPIO_NUM_6
#define UPPER_RIGHT_SEGMENT GPIO_NUM_7
#define MIDDLE_SEGMENT GPIO_NUM_8
#define LOWER_LEFT_SEGMENT GPIO_NUM_9
#define LOWER_RIGHT_SEGMENT GPIO_NUM_10
#define BOTTOM_SEGMENT GPIO_NUM_11

#define SLEEP_MICROSECONDS 1

#define METERS_TO_FEET_CONVERSION_FACTOR 3.28084
#define FEET_TO_MILES_CONVERSION_FACTOR 5280.0
#define SECONDS_TO_HOURS_CONVERSION_FACTOR 3600.0

#define COUNT_SPEED_THRESHHOLD 3.0

#define UART_BUFFER_SIZE 4096

static void set_low(const int* pins, int length)
{
	for (int i = 0; i < length; i++)
	{
		gpio_set_level(pins[i], 0);
	}
}

static void set_high(const int* pins, int length)
{
	for (int i = 0; i < length; i++)
	{
		gpio_set_level(pins[i], 1);
	}
}

static void set_segment_display(int display, int digit)
{
	gpio_set_level(MAX_SPEED_LOW_DIGIT, 0);
	gpio_set_level(MAX_SPEED_HIGH_DIGIT, 0);
	gpio_set_level(AVERAGE_SPEED_LOW_DIGIT, 0);
	gpio_set_level(AVERAGE_SPEED_HIGH_DIGIT, 0);
	gpio_set_level(display, 1);
	switch (digit)
	{
		case 0:
		{
			set_high((int[]) {TOP_SEGMENT, BOTTOM_SEGMENT, UPPER_LEFT_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_LEFT_SEGMENT, LOWER_RIGHT_SEGMENT}, 6);
			set_low((int[]) {MIDDLE_SEGMENT}, 1);
			break;
		}
		case 1:
		{
			set_high((int[]) {UPPER_RIGHT_SEGMENT, LOWER_RIGHT_SEGMENT}, 2);
			set_low((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_LEFT_SEGMENT, LOWER_LEFT_SEGMENT}, 5);
			break;
		}
		case 2:
		{
			set_high((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_LEFT_SEGMENT}, 5);
			set_low((int[]) {UPPER_LEFT_SEGMENT, LOWER_RIGHT_SEGMENT}, 2);
			break;
		}
		case 3:
		{
			set_high((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_RIGHT_SEGMENT}, 5);
			set_low((int[]) {UPPER_LEFT_SEGMENT, LOWER_LEFT_SEGMENT}, 2);
			break;
		}
		case 4:
		{
			set_high((int[]) {UPPER_LEFT_SEGMENT, MIDDLE_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_RIGHT_SEGMENT}, 4);
			set_low((int[]) {TOP_SEGMENT, BOTTOM_SEGMENT, LOWER_LEFT_SEGMENT}, 3);
			break;
		}
		case 5:
		{
			set_high((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_LEFT_SEGMENT, LOWER_RIGHT_SEGMENT}, 5);
			set_low((int[]) {UPPER_RIGHT_SEGMENT, LOWER_LEFT_SEGMENT}, 2);
			break;
		}
		case 6:
		{
			set_high((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_LEFT_SEGMENT, LOWER_LEFT_SEGMENT, LOWER_RIGHT_SEGMENT}, 6);
			set_low((int[]) {UPPER_RIGHT_SEGMENT}, 1);
			break;
		}
		case 7:
		{
			set_high((int[]) {TOP_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_RIGHT_SEGMENT}, 3);
			set_low((int[]) {MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_LEFT_SEGMENT, LOWER_LEFT_SEGMENT}, 4);
			break;
		}
		case 8:
		{
			set_high((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, BOTTOM_SEGMENT, UPPER_LEFT_SEGMENT, LOWER_LEFT_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_RIGHT_SEGMENT}, 7);
			break;
		}
		case 9:
		{
			set_high((int[]) {TOP_SEGMENT, MIDDLE_SEGMENT, UPPER_LEFT_SEGMENT, UPPER_RIGHT_SEGMENT, LOWER_RIGHT_SEGMENT}, 5);
			set_low((int[]) {BOTTOM_SEGMENT, LOWER_LEFT_SEGMENT}, 2);
			break;
		}
	}
}

float sexagesimal_to_radians(int degrees, float minutes)
{
	return (((float) degrees) + minutes / 60) * PI / 180.0;
}

float calculate_distance(float lat1, float long1, float lat2, float long2)
{
	float r = 2.0 * (float) EARTH_RADIUS_FEET * asin(pow(pow(sin((lat2-lat1)/2), 2) + cos(lat1) * cos(lat2) * pow(sin((long2 - long1) / 2), 2), .5));
	if (r < 0)
	{
		r *= -1;
	}
	return r;
}

void* thread_func(void* v)
{
	gpio_config_t io_conf = 
	{
		.pin_bit_mask = 
			(1ULL << MAX_SPEED_LOW_DIGIT) +
			(1ULL << MAX_SPEED_HIGH_DIGIT) +
			(1ULL << AVERAGE_SPEED_LOW_DIGIT) +
			(1ULL << AVERAGE_SPEED_HIGH_DIGIT) +
			(1ULL << TOP_SEGMENT) +
			(1ULL << UPPER_LEFT_SEGMENT) +
			(1ULL << UPPER_RIGHT_SEGMENT) +
			(1ULL << MIDDLE_SEGMENT) +
			(1ULL << LOWER_LEFT_SEGMENT) +
			(1ULL << LOWER_RIGHT_SEGMENT) +
			(1ULL << BOTTOM_SEGMENT),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE
	};

	gpio_config(&io_conf);

	float** args = (float**) v;
	float* average_speed = args[AVERAGE_SPEED];
	float* max_speed = args[MAX_SPEED];

	while (1)
	{
		int max = (int) *max_speed;
		int avg = (int) *average_speed;

		set_segment_display(MAX_SPEED_LOW_DIGIT, max % 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);

		set_segment_display(MAX_SPEED_HIGH_DIGIT, max / 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);

		set_segment_display(AVERAGE_SPEED_LOW_DIGIT, avg % 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);

		set_segment_display(AVERAGE_SPEED_HIGH_DIGIT, avg / 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);
	}

	return NULL;
}

void process_data_from_gps_sensor(int (*read_sensor_data)(uart_port_t, void*, uint32_t, uint32_t))
{
	if (read_sensor_data == NULL)
	{
		return;
	}

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
		size_t length = (*read_sensor_data)(UART_NUM_2, (uint8_t*) &buf[0], UART_BUFFER_SIZE, pdMS_TO_TICKS(100));
		
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
								if (seconds_passed != 0)
								{
									float speed = (distance / seconds_passed) * (FEET_TO_MILES_CONVERSION_FACTOR / SECONDS_TO_HOURS_CONVERSION_FACTOR);
									
									if (speed > *max_speed)
									{
										*max_speed = speed;
									}

									if (speed > COUNT_SPEED_THRESHHOLD)
									{
										float new_average_speed = *average_speed * (num_data_points - 1) / num_data_points + speed / num_data_points;
										*average_speed = new_average_speed;
										num_data_points++;
									}
								}
							}
							
							last_time = current_time;
							last_lat = current_lat;
							last_long = current_long;
							last_alt = current_alt;
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
