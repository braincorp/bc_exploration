#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "exploration/astar.h"
#include "safe_array.h"

namespace py = pybind11;

PYBIND11_MODULE(_exploration_module, m) {
  m.doc() = "python bindings for the exploration c++ code";
  m.def("check_for_collision2", &check_for_collision2);
//  m.def("oriented_astar", &oriented_astar, "Oriented A* algorithm, does not plan on angular space, rather has assigned angles for each movement direction.",
//        py::arg("start"),
//        py::arg("goal"),
//        py::arg("occupancy_map"),
//        py::arg("map_shape"),
//        py::arg("planning_scale"),
//        py::arg("footprint_masks"),
//        py::arg("mask_angles"),
//        py::arg("mask_radius"),
//        py::arg("outline_coords"),
//        py::arg("num_coords"),
//        py::arg("obstacle_values"),
//        py::arg("num_obstacle_values"),
//        py::arg("delta"),
//        py::arg("epsilon"),
//        py::arg("allow_diagonal"));
//  m.def("get_astar_angles", &get_astar_angles, "get angles used for astar");
}