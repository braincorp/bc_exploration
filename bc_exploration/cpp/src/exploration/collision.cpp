#include "../../inc/exploration/collision.h"
#include "../../inc/exploration/util.h"

extern "C" bool check_for_collision(const int* position, const uint8_t* occupancy_map, const int* map_shape,
                                    const bool* footprint_mask, const int mask_radius,
                                    const int* outline_coords, const int num_coords,
                                    const uint8_t* obstacle_values, const int num_obstacle_values) {
  /* Collision checking for custom footprints.
   *
   * :param position: 1x2 array with robot coordinate [row, column]
   * :param occupancy_map: 1xN*M array of flattened occupancy map
   * :param map_shape: 1x2 array with (N, M), the shape of the occupancy map
   * :param footprint_mask: (array(N*N)[int]) mask of the footprint rotated at the corresponding angle
   *                        needed for checking, i.e state[2]. N is 2 * mask_radius + 1,
   *                        the values are -1 for not footprint, 0 for footprint. (note the (N,N) mask is flattened)
   * :param mask_radius: (int) (footprint_mask.shape[0] - 1) / 2 the radius of the mask in pixels
   * :param outline_coords: (array(N*2)[int]) the coordinates that define the outline of the footprint. N is the number
   *                        of points that define the outline of the footprint. (it is a (N,2) array but flattened)
   * :param num_coords: number of coordinates in that define the outline of the footprint. i.e N from the description
   *                    of outline_coords
   * :param obstacle_values: (array(N)[uint8]) an array containing values that the collision checker should deem as an obstacle
   *                         i.e [127, 0]
   * :param num_obstacle values: the number of obstacle values given in the obstacle_values array.
   *
   * :return: (bool) whether there is a collision or not
   */

  // check if footprint it out of bounds -- if so, it is a collision
  for(int i = 0; i < 2 * num_coords; i += 2) {
    const int row = outline_coords[i] + position[0];
    const int col = outline_coords[i+1] + position[1];
    if (row < 0 || row >= map_shape[0]) {
      return true;
    }

    if (col < 0 || col >= map_shape[1]) {
      return true;
    }
  }

  int clipped_min_range[2], clipped_max_range[2];
  const int min_range[2] = {position[0] - mask_radius, position[1] - mask_radius};
  const int max_range[2] = {position[0] + mask_radius, position[1] + mask_radius};
  clip_range(min_range, max_range, map_shape, clipped_min_range, clipped_max_range);

  int mask_shape[2] = {2 * mask_radius + 1, 2 * mask_radius + 1};
  for (int m = clipped_min_range[0] - min_range[0], i = clipped_min_range[0]; i < clipped_max_range[0] + 1; m++, i++) {
    for(int n = clipped_min_range[1] - min_range[1], j = clipped_min_range[1]; j < clipped_max_range[1] + 1; n++, j++) {
      const int mask_idx = index_2d_to_1d(m, n, mask_shape);
      const int map_idx = index_2d_to_1d(i, j, map_shape);
      if (footprint_mask[mask_idx]) {
        for (int k = 0; k < num_obstacle_values; k++){
          if (occupancy_map[map_idx] == obstacle_values[k]) {
            return true;
          }
        }
      }
    }
  }

  return false;
}