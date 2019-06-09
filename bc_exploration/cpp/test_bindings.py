from __future__ import print_function, absolute_import, division

from bc_exploration import exploration_cpp


def test_bindings():
    assert exploration_cpp.__doc__
    assert exploration_cpp.c_astar.__doc__
    assert exploration_cpp.c_oriented_astar.__doc__
    assert exploration_cpp.c_get_astar_angles.__doc__
    assert exploration_cpp.c_check_for_collision.__doc__
