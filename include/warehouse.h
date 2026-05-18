#ifndef WAREHOUSE_H
#define WAREHOUSE_H

/*
 * warehouse.h
 * -----------
 * Declares the API for the dynamic 2D warehouse tile grid.
 * The grid is heap-allocated row-by-row at runtime based on user-specified
 * dimensions, satisfying the project's dynamic matrix requirement.
 *
 * Depends on: types.h
 */

#include "types.h"

/*
 * create_warehouse
 * ----------------
 * Allocates a Warehouse and its grid[height][width] tile matrix.
 * Each row is separately calloc'd; TILE_FLOOR (0) fills all cells.
 *
 * Parameters:
 *   width  - number of columns (must be > 0)
 *   height - number of rows    (must be > 0)
 *
 * Returns:
 *   Pointer to heap-allocated Warehouse. Exits on any allocation failure.
 */
Warehouse *create_warehouse(int width, int height);

/*
 * destroy_warehouse
 * -----------------
 * Frees every row of wh->grid, then the row-pointer array, then the struct.
 * This exact free order prevents memory leaks in the 2D allocation.
 *
 * Parameters:
 *   wh - Warehouse to free; no-op if NULL.
 */
void destroy_warehouse(Warehouse *wh);

/*
 * get_tile
 * --------
 * Returns the TileType at column x, row y.
 * Caller must ensure (x, y) is within bounds (see is_valid_pos).
 *
 * Parameters:
 *   wh - valid, non-NULL Warehouse*
 *   x  - column index
 *   y  - row index
 *
 * Returns: TileType value stored at grid[y][x].
 */
TileType get_tile(const Warehouse *wh, int x, int y);

/*
 * set_tile
 * --------
 * Writes type into grid[y][x].
 * Caller must ensure (x, y) is within bounds.
 *
 * Parameters:
 *   wh   - valid, non-NULL Warehouse*
 *   x    - column index
 *   y    - row index
 *   type - TileType to assign
 */
void set_tile(Warehouse *wh, int x, int y, TileType type);

/*
 * is_walkable
 * -----------
 * Returns 1 if the tile at (x, y) is not TILE_WALL (i.e., robots may
 * traverse it); returns 0 for walls and out-of-bounds coordinates.
 *
 * Parameters:
 *   wh - valid Warehouse*
 *   x  - column index
 *   y  - row index
 */
int is_walkable(const Warehouse *wh, int x, int y);

/*
 * is_valid_pos
 * ------------
 * Returns 1 if (x, y) lies within [0, width) x [0, height), 0 otherwise.
 *
 * Parameters:
 *   wh - valid Warehouse*
 *   x  - column index
 *   y  - row index
 */
int is_valid_pos(const Warehouse *wh, int x, int y);

/*
 * print_warehouse
 * ---------------
 * Prints a labelled ASCII map of the warehouse to stdout.
 * Characters: W=wall  F=floor  S=shelf  P=packing  C=charging
 *
 * Parameters:
 *   wh - valid Warehouse*
 */
void print_warehouse(const Warehouse *wh);

#endif /* WAREHOUSE_H */
