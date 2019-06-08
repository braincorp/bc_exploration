#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "exploration/astar.h"
#include "safe_array.h"

PYBIND11_MODULE(exploration_cpp, m) {
  m.doc() = "python bindings for the bc_exploration c++ code";
  m.def("c_check_for_collision", &check_for_collision);
  m.def("c_astar", &astar, "A* algorithm");
  m.def("c_oriented_astar", &oriented_astar, "Oriented A* algorithm, does not plan on angular space, rather has assigned angles for each movement direction.");
  m.def("c_get_astar_angles", &get_astar_angles, "get angles used for astar");
}