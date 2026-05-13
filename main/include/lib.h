#ifndef LIB_H
#define LIB_H

#include <stdint.h>
#include "driver/uart.h"

#define MAX_SPEED 0
#define AVERAGE_SPEED 1


float sexagesimal_to_radians(int degrees, float minutes);

// expeects radians
float calculate_distance(float lat1, float long1, float lat2, float long2);

void* thread_func(void* v);

void process_data_from_gps_sensor(int (*read_sensor_data)(uart_port_t, void*, uint32_t, uint32_t));

#endif
