#include <stdlib.h>
#include <math.h>
#include <unistd.h>

#include "lib.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

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
		set_segment_display(MAX_SPEED_LOW_DIGIT, ((int) *max_speed) % 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);

		set_segment_display(MAX_SPEED_HIGH_DIGIT, ((int) *max_speed) / 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);

		set_segment_display(AVERAGE_SPEED_LOW_DIGIT, ((int) *average_speed) % 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);

		set_segment_display(AVERAGE_SPEED_HIGH_DIGIT, ((int) *average_speed) / 10);
		esp_rom_delay_us(SLEEP_MICROSECONDS);
	}

	return NULL;
}
