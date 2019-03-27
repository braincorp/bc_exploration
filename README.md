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
   176 ../envs/grid_world.py
   231 ../envs/map_maker.py
   247 ../envs/test_grid_world.py
   289 ../algorithms/frontier_based_exploration.py
    27 ../algorithms/test_frontier_based_exploration.py
    36 ../integration/test_frontier_online_mapping_integration.py
   173 ../integration/frontier_online_mapping_integration.py
    21 ../agents/agent.py
   604 ../agents/frontier_agent.py
    46 ../agents/test_frontier_agent.py
    22 ../sensors/test_sensor_util.py
   232 ../sensors/sensors.py
   127 ../sensors/sensor_util.py
   217 ../footprints/footprints.py
    46 ../footprints/test_footprints.py
   119 ../benchmarks/benchmarks.py
    45 ../mapping/test_log_odds_mapper.py
   135 ../mapping/costmap.py
   167 ../mapping/log_odds_mapper.py
    70 ../mapping/test_costmap.py
    39 ../utilities/test_util.py
   145 ../utilities/util.py
    11 ../utilities/paths.py
    94 ../planners/test_planners.py
   427 ../planners/planners.py
  3746 total
```
