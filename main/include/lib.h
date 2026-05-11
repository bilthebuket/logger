#ifndef LIB_H
#define LIB_H

float sexagesimal_to_radians(int degrees, float minutes);

// expeects radians
float calculate_distance(float lat1, float long1, float lat2, float long2);

void set_max_speed(int speed);
void set_average_speed(int speed);

#endif
