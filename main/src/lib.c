#include <stdlib.h>
#include <math.h>
#include "lib.h"
#include "driver/gpio.h"

#define PI 3.141592653
#define EARTH_RADIUS_FEET 20903520

#define MAX_SPEED_LOW_DIGIT (int[]){GPIO_NUM_1, GPIO_NUM_2, GPIO_NUM_3}
#define MAX_SPEED_HIGH_DIGIT (int[]){GPIO_NUM_4, GPIO_NUM_5, GPIO_NUM_6}
#define AVERAGE_SPEED_LOW_DIGIT (int[]){GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_9}
#define AVERAGE_SPEED_HIGH_DIGIT (int[]){GPIO_NUM_10, GPIO_NUM_11, GPIO_NUM_12}

static void set_segment_display(const int* display, int digit);

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

void set_max_speed(int speed)
{
	int low = speed % 10;
	int high = speed / 10;

	set_segment_display(MAX_SPEED_LOW_DIGIT, low);
	set_segment_display(MAX_SPEED_HIGH_DIGIT, high);
}

void set_average_speed(int speed)
{
	int low = speed % 10;
	int high = speed / 10;

	set_segment_display(AVERAGE_SPEED_LOW_DIGIT, low);
	set_segment_display(AVERAGE_SPEED_HIGH_DIGIT, high);
}

static void set_segment_display(const int* io_pins, int digit)
{
	if (digit >= 4)
	{
		gpio_set_level(io_pins[2], 1);
		digit -= 4;
	}
	else
	{
		gpio_set_level(io_pins[2], 0);
	}

	if (digit >= 2)
	{
		gpio_set_level(io_pins[1], 1);
		digit -= 2;
	}
	else
	{
		gpio_set_level(io_pins[1], 0);
	}

	if (digit >= 1)
	{
		gpio_set_level(io_pins[0], 1);
	}
	else
	{
		gpio_set_level(io_pins[0], 0);
	}
}
