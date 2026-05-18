#ifndef PATHFINDING_H
#define PATHFINDING_H

/*
 * pathfinding.h
 * -------------
 * Declares the BFS-based spatial search algorithms used by robots to
 * determine travel distances and locate the nearest target tile type
 * on the dynamic warehouse grid.
 *
 * Both functions treat all non-TILE_WALL tiles as walkable, reflecting
 * the real constraint that robots must avoid walls only.
 *
 * Depends on: types.h, warehouse.h
 */

#include "types.h"
#include "warehouse.h"

/*
 * bfs_distance
 * ------------
 * Computes the minimum number of grid steps (Manhattan-compatible BFS)
 * from start to end, traversing only non-wall tiles.
 *
 * All working memory (visited array, BFS queue) is heap-allocated to
 * prevent stack overflow on large maps.
 *
 * Parameters:
 *   wh    - the warehouse grid (read-only)
 *   start - BFS origin position
 *   end   - destination position
 *
 * Returns:
 *   Non-negative integer step count on success.
 *   0  if start == end.
 *   -1 if no walkable path exists (e.g. end is walled off).
 */
int bfs_distance(const Warehouse *wh, Position start, Position end);

/*
 * find_nearest_tile_bfs
 * ---------------------
 * Locates the nearest reachable tile of target_type from start via BFS.
 * If the start position itself has the target type it is returned immediately.
 *
 * Parameters:
 *   wh          - the warehouse grid (read-only)
 *   start       - search origin
 *   target_type - TileType to find (e.g. TILE_CHARGING)
 *
 * Returns:
 *   Position of the nearest reachable matching tile.
 *   {-1, -1} if no such tile is reachable from start.
 */
Position find_nearest_tile_bfs(const Warehouse *wh, Position start,
                                TileType target_type);

#endif /* PATHFINDING_H */
