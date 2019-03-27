#pragma once

#include <iostream>

extern "C" bool check_for_collision(const int* position, const uint8_t* occupancy_map, const int* map_shape,
                                    const bool* footprint_mask, int mask_radius,
                                    const int* outline_coords, int num_coords,
                                    const uint8_t* obstacle_values, int num_obstacle_values);