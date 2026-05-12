#ifndef LIB_H
#define LIB_H

#define MAX_SPEED 0
#define AVERAGE_SPEED 1

float sexagesimal_to_radians(int degrees, float minutes);

// expeects radians
float calculate_distance(float lat1, float long1, float lat2, float long2);

void* thread_func(void* v);

#endif
