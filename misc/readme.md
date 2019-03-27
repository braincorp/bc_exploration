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
