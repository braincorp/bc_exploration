# README
environments and algorithms for exploration

Implemented an exploration simulation framework with modularity in:
 - Sensors: Neighborhood, 2D-Lidar
 - Exploration Algorithms: Frontier-Based Exploration
 - Mapping: Log-Odds Mapping
 - Motion Planners: Weighted A*

Can explore any environment that is given as a costmap
Can easily plug in play other planners / mappers
Frontier-based exploration implemented with a few different behavioral modes, each one explores a bit differently.

Includes integration with online mapping.
Includes benchmarking framework / benchmarks


line count breakdown:
```
   166 ../benchmarks/benchmarks.py
   145 ../mapping/costmap.py
   188 ../mapping/log_odds_mapper.py
    38 ../mapping/test_costmap.py
    45 ../mapping/test_log_odds_mapper.py
    39 ../utilities/test_util.py
    37 ../utilities/paths.py
    76 ../utilities/visualization.py
    39 ../utilities/map_transformations.py
   209 ../utilities/util.py
    49 ../setup.py
   288 ../sensors/sensors.py
   130 ../sensors/sensor_util.py
    23 ../sensors/test_sensor_util.py
   220 ../envs/grid_world.py
   245 ../envs/map_maker.py
   247 ../envs/test_grid_world.py
   105 ../footprints/footprint_points.py
   311 ../footprints/footprints.py
    63 ../footprints/test_footprints.py
    57 ../footprints/collision_cpp.py
   180 ../planners/astar_cpp.py
   209 ../planners/test_astar_cpp.py
    75 ../agents/test_frontier_agent.py
    34 ../agents/agent.py
   459 ../agents/frontier_agent.py
    27 ../algorithms/test_frontier_based_exploration.py
   270 ../algorithms/frontier_based_exploration.py
    37 ../misc/bc_linters/check_illegal_constructs.py
   373 ../misc/bc_linters/docstring_checker.py
    93 ../misc/bc_linters/check_illegal_imports.py
  4477 total
```
