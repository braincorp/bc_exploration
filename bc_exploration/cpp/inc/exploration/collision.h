#pragma once

#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "safe_array.h"

extern "C" bool check_for_collision(const int* position, const uint8_t* occupancy_map, const int* map_shape,
                                    const bool* footprint_mask, int mask_radius,
                                    const int* outline_coords, int num_coords,
                                    const uint8_t* obstacle_values, int num_obstacle_values);


bool check_for_collision2(pybind11::safe_array<int, 1> position,
                         pybind11::safe_array<uint8_t, 2> occupancy_map,
                         pybind11::safe_array<bool, 2> footprint_mask,
                         pybind11::safe_array<int, 2> outline_coords,
                         pybind11::safe_array<uint8_t, 1> obstacle_values);