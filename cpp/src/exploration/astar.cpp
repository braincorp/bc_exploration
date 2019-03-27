#include "../../inc/exploration/astar.h"
#include "../../inc/exploration/util.h"

extern "C" bool astar(const int* start, const int* goal,
                      const uint8_t* occupancy_map, const int* map_shape, const int planning_scale,
                      const uint8_t* obstacle_values, const int num_obstacle_values,
                      const float delta, const float epsilon, const bool allow_diagonal,
                      int* path) {
  /* A* algorithm
   *
   * :param start: 1x2 array with start coordinate [row, column]
   * :param goal: 1x2 array with goal coordinate [row, column]
   * :param occupancy_map: 1xN*M array of flattened occupancy map
   * :param map_shape: 1x2 array with (N, M), the shape of the occupancy map
   * :param planning_scale: value > 1, to plan on a lower resolution than the original occupancy map resolution,
   *                        this value is round_int(desired resolution / original resolution)
   * :param mask_size: size of the mask U described above
   * :param epsilon: weighting for the heuristic in the A* algorithm
   * :param allow_diagonal: whether to allow diagonal movements
   * :return path: this function will populate path with the 1d indices of the path from start to goal
   */

  // verify planning scale is correct
  if (planning_scale < 1) {
    throw std::logic_error("ERROR: parameter planning_scale of c++ function oriented_astar() must be greater than 1 ");
  }

  const float inf = std::numeric_limits<float>::infinity();

  const int start_idx = index_2d_to_1d(start, map_shape);
  const int goal_idx = index_2d_to_1d(goal, map_shape);

  for(int i = 0; i < num_obstacle_values; i++){
    if (occupancy_map[start_idx] == obstacle_values[i] || occupancy_map[goal_idx] == obstacle_values[i]) {
      return false;
    }
  }

  Node start_node = Node(0.0, start_idx);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node> > open_set;
  open_set.push(start_node);

  std::vector<float> costs((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    costs[i] = inf;
  }
  costs[start_idx] = 0.0;

  std::vector<int> paths((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    paths[i] = -1;
  }

  int children[8][2];
  int parent_coord[2];
  int solution_idx = -1;

  bool is_successful = false;
  while(!open_set.empty()) {
    Node parent = open_set.top();
    open_set.pop();

    index_1d_to_2d(parent.idx, map_shape, parent_coord);

    float distance_to_goal = euclidean(parent_coord, goal);
    if (distance_to_goal <= delta || (planning_scale != 1 && distance_to_goal <= planning_scale + delta)) {
      is_successful = true;
      solution_idx = parent.idx;
      break;
    }

    children[0][0] = parent_coord[0];                  children[0][1] = parent_coord[1] - planning_scale;
    children[1][0] = parent_coord[0] + planning_scale; children[1][1] = parent_coord[1] - planning_scale;
    children[2][0] = parent_coord[0] + planning_scale; children[2][1] = parent_coord[1];
    children[3][0] = parent_coord[0] + planning_scale; children[3][1] = parent_coord[1] + planning_scale;
    children[4][0] = parent_coord[0];                  children[4][1] = parent_coord[1] + planning_scale;
    children[5][0] = parent_coord[0] - planning_scale; children[5][1] = parent_coord[1] + planning_scale;
    children[6][0] = parent_coord[0] - planning_scale; children[6][1] = parent_coord[1];
    children[7][0] = parent_coord[0] - planning_scale; children[7][1] = parent_coord[1] - planning_scale;

    for (int c = 0; c < 8; c++) {
      // if diagonal is not allowed, skip those children
      if (!allow_diagonal && c % 2 != 0) {
            continue;
      }

      // skip child if out of bounds
      if (children[c][0] < 0 || children[c][0] >= map_shape[0] \
              || children[c][1] < 0 || children[c][1] >= map_shape[1]) {
        continue;
      }

      int child_idx = index_2d_to_1d(children[c], map_shape);

      // skip child if it lies on an obstacle
      bool on_obstacle = false;
      for(int i = 0; i < num_obstacle_values; i++) {
        if (occupancy_map[child_idx] == obstacle_values[i]) {
          on_obstacle = true;
        }
      }
      if (on_obstacle) {
        continue;
      }

      float g = costs[parent.idx] + euclidean(parent_coord, children[c]);


      if (costs[child_idx] > g) {
        costs[child_idx] = g;
        paths[child_idx] = parent.idx;

        float f = g + epsilon * euclidean(children[c], goal);
        open_set.push(Node(f, child_idx));
      }
    }
  }

  int current_idx = solution_idx;
  std::stack<int> path_stack;
  while(start_idx != current_idx) {
    path_stack.push(current_idx);
    current_idx = paths[current_idx];
  }

  int i = 0;
  while(!path_stack.empty()) {
    path[i] = path_stack.top();
    path_stack.pop();
    i++;
  }

  return is_successful;
}


extern "C" void get_astar_angles(float* angles) {

  // these angles correspond to the x,y world converted
  // angles of moving in the corresponding
  // children direction in the astar algorithm
  // see astar assignment of children.
  angles[0] = (float) -M_PI;
  angles[1] = (float) (-3.*M_PI_4);
  angles[2] = (float) -M_PI_2;
  angles[3] = (float) -M_PI_4;
  angles[4] = (float) 0.;
  angles[5] = (float) M_PI_4;
  angles[6] = (float) M_PI_2;
  angles[7] = (float) (3.*M_PI_4);
}


extern "C" bool oriented_astar(const int* start, const int* goal,
                               const uint8_t* occupancy_map, const int* map_shape, const int planning_scale,
                               const bool* footprint_masks, const float* mask_angles, const int mask_radius,
                               const int* outline_coords, const int num_coords,
                               const uint8_t* obstacle_values, const int num_obstacle_values,
                               const float delta, const float epsilon, const bool allow_diagonal,
                               int* path_idxs, float* path_angles) {
  /* Oriented A* algorithm, does not plan on angular space, rather has assigned angles for each movement direction.
   *
   * :param start: 1x3 array with start coordinate [row, column]
   * :param goal: 1x3 array with goal coordinate [row, column]
   * :param occupancy_map: 1xN*M array of flattened occupancy map
   * :param map_shape: 1x2 array with (N, M), the shape of the occupancy map,
   * :param planning_scale: value > 1, to plan on a lower resolution than the original occupancy map resolution,
   *                       this value is round_int(desired resolution / original resolution)
   * :param footprint_masks: 1xU*8 where U is the size of a UxU mask for checking footprint collision,
   *                         where U must be odd, such that U / 2 + 1 is the center pixel,
   *                         the mask is true for footprint and false for not footprint,
   *                         which is centered among the center pixel in the mask
   * :param mask_angles: 1x8 array of these angles (in order) [-pi, -3pi/4, -pi/2, -pi/4, 0, pi/4, pi/2, 3pi/4]
   *                     can use the "get_astar_angles function to get these angles.
   * :param mask_size: size of the mask U described above
   * :param costmap_scale: multiplier of which to downscale planning by. I.E if original map is 0.03 resolution, and
   *                       we wish to plan on a 0.1 resolution, costmap_scale = 0.1 / 0.03 = 3.333,
   *                       if costmap_scale is 1 no downscaling is performed.
   * :param free_value: value in occupancy_map which is traversable/free
   * :param epsilon: weighting for the heuristic in the A* algorithm
   * :param allow_diagonal: whether to allow diagonal movements
   * :return path: this function will populate path with the 1d indices of the path from start to goal
   */

  std::cout << "start plan, ";

  // todo: we arent going to goal angle.. we need to make sure we collision check it if we want to.
  const float inf = std::numeric_limits<float>::infinity();

  // verify angles are correct (i.e human knows what he is doing when he calls this function)
  float correct_angles[8];
  get_astar_angles(correct_angles);
  for (int i = 0; i < 8; i++) {
    if (correct_angles[i] != mask_angles[i]) {
      throw std::logic_error("ERROR: parameter mask_angles of c++ function oriented_astar() does not match required angles. "
                             "See get_astar_angles() for the correct angles/order. "
                             "Note, the footprint masks must match these angles, or you will get undesired behavior!");
    }
  }

  // verify planning scale is correct
  if (planning_scale < 1) {
    throw std::logic_error("ERROR: parameter planning_scale of c++ function oriented_astar() must be greater than 1 ");
  }

  const int start_idx = index_2d_to_1d(start, map_shape);

  Node start_node = Node(0.0, start_idx);

  std::priority_queue<Node, std::vector<Node>, std::greater<Node> > open_set;
  open_set.push(start_node);

  std::vector<float> costs((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    costs[i] = inf;
  }
  costs[start_idx] = 0.0;

  std::vector<int> paths((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    paths[i] = -1;
  }

  std::vector<int> paths_angle_inds((ulong) map_shape[0]*map_shape[1]);
  for (int i = 0; i < map_shape[0]*map_shape[1]; i++) {
    paths_angle_inds[i] = -1;
  }

  const int mask_size = 2 * mask_radius + 1;

  int children[8][2];
  int parent_coord[2];

  int solution_idx = -1;
  float closest_distance_to_goal = inf;

  int num_nodes_expanded = 0;
  bool is_successful = false;
  while(!open_set.empty()) {
    Node parent = open_set.top();
    open_set.pop();

    index_1d_to_2d(parent.idx, map_shape, parent_coord);

    float distance_to_goal = euclidean(parent_coord, goal);

    if (distance_to_goal < closest_distance_to_goal) {
      closest_distance_to_goal = distance_to_goal;
      solution_idx = parent.idx;
    }

    if (distance_to_goal <= delta || (planning_scale != 1 && distance_to_goal <= planning_scale + delta)) {
      is_successful = true;
      solution_idx = parent.idx;
      break;
    }

    // todo Note that if planning_scale is too large, in theory this will cause us to jump over thin obstacles.
    // todo In reality, this will never happen, since we won't be planning on a large enough resolutions.
    // todo can I somehow efficiently check the line between the pose and the neighbor to make sure the move is valid?
    children[0][0] = parent_coord[0];                  children[0][1] = parent_coord[1] - planning_scale;
    children[1][0] = parent_coord[0] + planning_scale; children[1][1] = parent_coord[1] - planning_scale;
    children[2][0] = parent_coord[0] + planning_scale; children[2][1] = parent_coord[1];
    children[3][0] = parent_coord[0] + planning_scale; children[3][1] = parent_coord[1] + planning_scale;
    children[4][0] = parent_coord[0];                  children[4][1] = parent_coord[1] + planning_scale;
    children[5][0] = parent_coord[0] - planning_scale; children[5][1] = parent_coord[1] + planning_scale;
    children[6][0] = parent_coord[0] - planning_scale; children[6][1] = parent_coord[1];
    children[7][0] = parent_coord[0] - planning_scale; children[7][1] = parent_coord[1] - planning_scale;

    for (int c = 0; c < 8; c++) {

      // if diagonal is not allowed, skip those children
      if (!allow_diagonal && c % 2 != 0) {
        continue;
      }

      // skip child if out of bounds
      if (children[c][0] < 0 || children[c][0] >= map_shape[0] \
          || children[c][1] < 0 || children[c][1] >= map_shape[1]) {
        continue;
      }

      if (check_for_collision(children[c], occupancy_map, map_shape,
                              footprint_masks + (c * mask_size * mask_size), mask_radius,
                              outline_coords + (c * 2 * num_coords), num_coords,
                              obstacle_values, num_obstacle_values)) {
        continue;
      }

      float g = costs[parent.idx] + euclidean(parent_coord, children[c]);

      int child_idx = index_2d_to_1d(children[c], map_shape);
      if (costs[child_idx] > g) {
        costs[child_idx] = g;
        paths[child_idx] = parent.idx;
        paths_angle_inds[child_idx] = c;

        float f = g + epsilon * euclidean(children[c], goal);
        open_set.push(Node(f, child_idx));
      }
    }
    num_nodes_expanded++;
  }

  int current_idx = solution_idx;
  float current_angle = mask_angles[paths_angle_inds[current_idx]];
  std::stack<std::pair<int, float> > path_stack;
  while(start_idx != current_idx) {
    path_stack.push(std::make_pair(current_idx, current_angle));
    current_idx = paths[current_idx];
    current_angle = mask_angles[paths_angle_inds[current_idx]];
  }

  int i = 0;
  std::pair<int, float> item;
  while(!path_stack.empty()) {
    item = path_stack.top();
    path_idxs[i] = item.first;
    path_angles[i] = item.second;
    path_stack.pop();
    i++;
  }

  std::cout << "expanded " << num_nodes_expanded << " nodes. successful: " << is_successful << std::endl;
  return is_successful;
}