"""astar_cpp.py
Contains python wrappers for c++ code for astar planning.
"""
from __future__ import print_function, absolute_import, division

import ctypes
import numpy as np
from utilities.paths import get_exploration_so_path
from utilities.util import xy_to_rc, rc_to_xy

int_1d_type = np.ctypeslib.ndpointer(dtype=np.int32, ndim=1, flags='C_CONTIGUOUS')
int_2d_type = np.ctypeslib.ndpointer(dtype=np.int32, ndim=2, flags='C_CONTIGUOUS')
float_1d_type = np.ctypeslib.ndpointer(dtype=np.float32, ndim=1, flags='C_CONTIGUOUS')
float_2d_type = np.ctypeslib.ndpointer(dtype=np.float32, ndim=2, flags='C_CONTIGUOUS')
uint8_1d_type = np.ctypeslib.ndpointer(dtype=np.uint8, ndim=1, flags='C_CONTIGUOUS')
uint8_2d_type = np.ctypeslib.ndpointer(dtype=np.uint8, ndim=2, flags='C_CONTIGUOUS')
bool_2d_type = np.ctypeslib.ndpointer(dtype=np.bool, ndim=2, flags='C_CONTIGUOUS')

lib = ctypes.cdll.LoadLibrary(get_exploration_so_path())
c_astar = lib.astar
c_astar.restype = ctypes.c_bool
c_astar.argtypes = [int_1d_type, int_1d_type,
                    uint8_2d_type, int_1d_type, ctypes.c_int32,
                    uint8_1d_type, ctypes.c_int32,
                    ctypes.c_float, ctypes.c_float, ctypes.c_bool,
                    int_1d_type]

c_get_astar_angles = lib.get_astar_angles
c_get_astar_angles.restype = None
c_get_astar_angles.argtypes = [float_1d_type]

c_oriented_astar = lib.oriented_astar
c_oriented_astar.restype = ctypes.c_bool
c_oriented_astar.argtypes = [int_1d_type, int_1d_type,
                             uint8_2d_type, int_1d_type, ctypes.c_int,
                             bool_2d_type, float_1d_type, ctypes.c_int32,
                             int_2d_type, ctypes.c_int32,
                             uint8_1d_type, ctypes.c_int32,
                             ctypes.c_float, ctypes.c_float, ctypes.c_bool,
                             int_1d_type, float_1d_type]


def get_astar_angles():
    """
    Return the angles used for astar.
    :return array(8)[float]: angles used for astar (in the correct order)
    """
    c_angles = np.zeros((8,), dtype=np.float32)
    c_get_astar_angles(c_angles)
    return c_angles


def astar(start, goal, occupancy_map, obstacle_values, planning_scale=1, delta=0.0, epsilon=1.0, allow_diagonal=False):
    """
    Wrapper for vanilla a-star c++ planning. given a start and end and a map, give a path.
    :param start array(3)[float]: [x, y, theta] start pose of the robot
    :param goal array(3)[float]: [x, y, theta] goal pose of the robot
    :param occupancy_map Costmap: occupancy map used for planning, data must be compatible with uint8
    :param obstacle_values array(N)[uint8]: an array containing values that the collision checker should deem as an obstacle
                             i.e [127, 0]
    :param planning_scale int: value > 1, to plan on a lower resolution than the original occupancy map resolution,
                           this value is round_int(desired resolution / original resolution)
    :param delta float: distance in pixels to extend the goal region for solution (allow to be within delta away from goal)
                  TODO FIX this to be in meters
    :param epsilon float: weighting for the heuristic in the A* algorithm
    :param allow_diagonal bool: whether to allow diagonal movements
    :return Tuple[bool, array(N, 3)[float]]: whether we were able to successfully plan to the goal node,
                                              and the most promising path to the goal node (solution if obtained)
    """
    start_px = xy_to_rc(start, occupancy_map)
    c_start = np.array(start_px[:2], dtype=np.int32)

    goal_px = xy_to_rc(goal, occupancy_map)
    c_goal = np.array(goal_px[:2], dtype=np.int32)

    c_map_shape = np.array(occupancy_map.data.shape, dtype=np.int32)
    c_path = -1 * np.ones((occupancy_map.data.shape[0] * occupancy_map.data.shape[1],), dtype=np.int32)
    c_occupancy_map = occupancy_map.data.astype(np.uint8)

    success = c_astar(c_start,
                      c_goal,
                      c_occupancy_map,
                      c_map_shape,
                      planning_scale,
                      obstacle_values,
                      obstacle_values.shape[0],
                      delta,
                      epsilon,
                      allow_diagonal,
                      c_path)
    end_idx = None
    for end_idx, path_idx in enumerate(c_path):
        if path_idx == -1:
            break

    path_px = c_path[:end_idx]
    path_px = np.dstack(np.unravel_index(path_px, occupancy_map.data.shape)).squeeze()
    path = rc_to_xy(path_px, occupancy_map)
    return success, np.vstack(([start], path))


def oriented_astar(start, goal, occupancy_map, footprint_masks, mask_radius,
                   outline_coords, obstacle_values, planning_scale=1, delta=0.0, epsilon=1.0, allow_diagonal=True):
    """
        Oriented Astar C++ wrapper for python. Formats input data in required format for c++ function, the calls it,
    returning the path if found.
    :param start array(3)[float]: [x, y, theta] start pose of the robot
    :param goal array(3)[float]: [x, y, theta] goal pose of the robot
    :param occupancy_map Costmap: occupancy map used for planning, data must be compatible with uint8
    :param footprint_masks array(N,M,M)[int]: masks of the footprint rotated at the corresponding angles
                            needed for checking, i.e state[2]. N is 2 * mask_radius + 1,
                            the values are -1 for not footprint, 0 for footprint.
                            N is the dimension across angles, (M, M) is the mask shape
    :param mask_radius int: (footprint_mask.shape[0] - 1) / 2 the radius of the mask in pixels
    :param outline_coords array(N, 2)[int]: the coordinates that define the outline of the footprint. N is the number
                           of points that define the outline of the footprint
    :param obstacle_values array(N)[uint8]: an array containing values that the collision checker should deem as an obstacle
                             i.e [127, 0]
    :param planning_scale int: value > 1, to plan on a lower resolution than the original occupancy map resolution,
                           this value is round_int(desired resolution / original resolution)
    :param delta float: distance in pixels to extend the goal region for solution (allow to be within delta away from goal)
                  TODO FIX this to be in meters
    :param epsilon float: weighting for the heuristic in the A* algorithm
    :param allow_diagonal bool: whether to allow diagonal movements
    :return Tuple[bool, array(N, 3)[float]]: whether we were able to successfully plan to the goal,
                                              and the most promising path to the goal node (solution if obtained)
    """
    c_angles = np.zeros((8,), dtype=np.float32)
    c_get_astar_angles(c_angles)

    start_px = xy_to_rc(start, occupancy_map)
    c_start = np.array(start_px[:2], dtype=np.int32)

    goal_px = xy_to_rc(goal, occupancy_map)
    c_goal = np.array(goal_px[:2], dtype=np.int32)

    c_occupancy_map = occupancy_map.data.astype(np.uint8)

    c_map_shape = np.array(occupancy_map.get_shape(), dtype=np.int32)
    c_path = -1 * np.ones((c_map_shape[0] * c_map_shape[1],), dtype=np.int32)
    c_path_angles = -1 * np.ones((c_map_shape[0] * c_map_shape[1],), dtype=np.float32)

    c_footprint_masks = np.logical_not(np.array(footprint_masks, dtype=np.bool))
    c_footprint_masks = c_footprint_masks.reshape((c_footprint_masks.shape[0],
                                                   c_footprint_masks.shape[1] * c_footprint_masks.shape[2]))
    c_footprint_masks = c_footprint_masks.astype(np.bool)

    c_outline_coords = np.array(outline_coords, dtype=np.int32)
    c_outline_coords = c_outline_coords.reshape((c_outline_coords.shape[0] * c_outline_coords.shape[1],
                                                 c_outline_coords.shape[2]))

    c_obstacle_values = np.array(obstacle_values, dtype=np.uint8)

    success = c_oriented_astar(c_start,
                               c_goal,
                               c_occupancy_map,
                               c_map_shape,
                               planning_scale,
                               c_footprint_masks,
                               c_angles,
                               mask_radius,
                               c_outline_coords,
                               outline_coords[0].shape[0],
                               c_obstacle_values,
                               c_obstacle_values.shape[0],
                               delta,
                               epsilon,
                               allow_diagonal,
                               c_path,
                               c_path_angles)
    end_idx = None
    for end_idx, path_idx in enumerate(c_path):
        if path_idx == -1:
            break

    path_px = c_path[:end_idx]
    path_px = np.dstack(np.unravel_index(path_px, occupancy_map.get_shape())).squeeze(0)
    path_px = np.hstack((path_px, np.expand_dims(c_path_angles[:end_idx], axis=1)))
    path = rc_to_xy(path_px, occupancy_map)
    return success, np.vstack(([start], path))
