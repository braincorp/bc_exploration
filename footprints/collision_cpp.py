"""collision_cpp.py
Contains python wrappers for c++ code for footprint collision checking.
"""
from __future__ import print_function, absolute_import, division

import ctypes
import numpy as np

from utilities.paths import get_exploration_so_path
from utilities.util import xy_to_rc

int_1d_type = np.ctypeslib.ndpointer(dtype=np.int32, ndim=1, flags='C_CONTIGUOUS')
int_2d_type = np.ctypeslib.ndpointer(dtype=np.int32, ndim=2, flags='C_CONTIGUOUS')
bool_2d_type = np.ctypeslib.ndpointer(dtype=np.bool, ndim=2, flags='C_CONTIGUOUS')
uint8_1d_type = np.ctypeslib.ndpointer(dtype=np.uint8, ndim=1, flags='C_CONTIGUOUS')
uint8_2d_type = np.ctypeslib.ndpointer(dtype=np.uint8, ndim=2, flags='C_CONTIGUOUS')

c_check_for_collision = ctypes.cdll.LoadLibrary(get_exploration_so_path()).check_for_collision
c_check_for_collision.restype = ctypes.c_bool
c_check_for_collision.argtypes = [int_1d_type, uint8_2d_type, int_1d_type, bool_2d_type, ctypes.c_int32,
                                  int_2d_type, ctypes.c_int32, uint8_1d_type, ctypes.c_int32]


def check_for_collision(state, occupancy_map, footprint_mask, mask_radius, outline_coords, obstacle_values):
    """
    Check if the current state with the given footprint is colliding or not.
    :param state array(3)[float]: pose of the robot
    :param occupancy_map Costmap: object to check the footprint against
    :param footprint_mask array(N,N)[int]: mask of the footprint rotated at the corresponding angle
                           needed for checking, i.e state[2]. N is 2 * mask_radius + 1,
                           the values are -1 for not footprint, 0 for footprint.
    :param mask_radius int: (footprint_mask.shape[0] - 1) / 2 the radius of the mask in pixels
    :param outline_coords array(N, 2)[int]: the coordinates that define the outline of the footprint. N is the number
                           of points that define the outline of the footprint
    :param obstacle_values array(N)[uint8]: an array containing values that the collision checker should deem as an obstacle
                             i.e [127, 0]
    :return bool: whether there is a collision or not
    """
    state_px = xy_to_rc(state, occupancy_map)
    c_state = np.array(state_px[:2], dtype=np.int32)
    c_occupancy_map = occupancy_map.data.astype(np.uint8)
    c_map_shape = np.array(occupancy_map.get_shape(), dtype=np.int32)
    c_footprint_mask = np.logical_not(np.array(footprint_mask, dtype=np.bool))
    c_outline_coords = np.array(outline_coords, dtype=np.int32)
    c_obstacle_values = np.array(obstacle_values, dtype=np.uint8)

    is_colliding = c_check_for_collision(c_state,
                                         c_occupancy_map,
                                         c_map_shape,
                                         c_footprint_mask,
                                         mask_radius,
                                         c_outline_coords,
                                         c_outline_coords.shape[0],
                                         c_obstacle_values,
                                         c_obstacle_values.shape[0])

    return is_colliding
