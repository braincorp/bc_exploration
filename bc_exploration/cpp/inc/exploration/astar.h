#pragma once

#include <iostream>
#include <queue>
#include <limits>
#include <cmath>
#include <stack>
#include <vector>
#include <stdexcept>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "collision.h"

struct Node {
  float f;
  int idx;

  Node(const float f, const int idx) : f(f), idx(idx) {}
};

bool operator>(const Node &n1, const Node &n2);

extern "C" bool astar(const int* start, const int* goal,
                      const uint8_t* occupancy_map, const int* map_shape, int planning_scale,
                      const uint8_t* obstacle_values, int num_obstacle_values,
                      float delta, float epsilon, bool allow_diagonal,
                      int* path);

std::vector<float> get_astar_angles();

std::tuple<bool, std::vector<int>, std::vector<float>> oriented_astar(const int* start, const int* goal,
                                              const uint8_t* occupancy_map, const int* map_shape,  int planning_scale,
                                              const bool* footprint_masks, const float* mask_angles, int mask_radius,
                                              const int* outline_coords, int num_coords,
                                              const uint8_t* obstacle_values, int num_obstacle_values,
                                              float delta, float epsilon, bool allow_diagonal);
