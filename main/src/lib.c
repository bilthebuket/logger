#include <math.h>
#include "lib.h"

#define PI 3.141592653
#define EARTH_RADIUS_FEET 20903520

float sexagesimal_to_radians(int degrees, float minutes)
{
	return (((float) degrees) + minutes / 60) * PI / 180.0;
}

float calculate_distance(float lat1, float long1, float lat2, float long2)
{
	return abs(2 * EARTH_RADIUS_FEET * asin(pow(pow(sin((lat2-lat1)/2), 2) + cos(lat1) * cos(lat2) * pow(sin((long2 - long1) / 2), 2), .5)));
}
