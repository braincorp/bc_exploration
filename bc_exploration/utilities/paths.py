"""paths.py
path based utility functions for exploration
"""

from __future__ import print_function, absolute_import, division

import os


def get_exploration_dir():
    """
    Get the exploration root dir
    :return str: exploration root dir
    """
    return os.path.dirname(os.path.dirname(__file__))


def get_maps_dir():
    """
    Get the exploration/maps dir
    :return str: exploration/maps dir
    """
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), "maps")


def get_exploration_so_path():
    """
    Get the exploration binary directory (with the .so) either from brain (if exists) or locally
    :return str: path to the exploration.so file used for planning with cpp planners
    """
    if not hasattr(get_exploration_so_path, 'local_so_path'):
        local_so_paths = [os.path.join(dp, f)
                          for dp, dn, filenames in os.walk(os.path.join(get_exploration_dir(), "../build/"))
                          for f in filenames if f == 'exploration.so']
        if local_so_paths:
            get_exploration_so_path.local_so_path = local_so_paths[0]
        else:
            raise OSError(local_so_paths[0] + "not found. please make sure source is compiled."
                                              "call make patch.exploration-cpp outside sandbox to build, "
                                              "or 'cmake .. && make' in cpp/build/ folder in exploration")

    return get_exploration_so_path.local_so_path
