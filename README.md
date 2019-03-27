# README
environments and algorithms for exploration
**It is now a git subtree within shining, but the repo still exists, and changes must be pushed/pulled to it as such: From the shining_software root call:**
```
# pull
git subtree pull --prefix src/shining_software/exploration git@github.com:braincorp/exploration.git brain_corp
# push
 git subtree push --prefix=src/shining_software/exploration git@github.com:braincorp/exploration.git brain_corp
```


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

Written with the goal of modularity towards the shining stack, such that if wanted, we can remove all dependencies on brain's code, and it will run without brain's code. The intention is to possibly release some of it as open source / share the code with research professors

**still missing some unit tests, I will add more tests in the next PR, but this needs to be merged so that Mark can make a PR**


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
   283 ../sensors/sensors.py
   130 ../sensors/sensor_util.py
    22 ../sensors/test_sensor_util.py
   208 ../envs/grid_world.py
   239 ../envs/map_maker.py
   247 ../envs/test_grid_world.py
    79 ../footprints/footprint_points.py
   311 ../footprints/footprints.py
    63 ../footprints/test_footprints.py
    57 ../footprints/collision_cpp.py
   180 ../planners/astar_cpp.py
   209 ../planners/test_astar_cpp.py
    74 ../agents/test_frontier_agent.py
    34 ../agents/agent.py
   458 ../agents/frontier_agent.py
    27 ../algorithms/test_frontier_based_exploration.py
   271 ../algorithms/frontier_based_exploration.py
  3874 total
```
