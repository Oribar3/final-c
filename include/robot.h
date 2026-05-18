#ifndef ROBOT_H
#define ROBOT_H

/*
 * robot.h
 * -------
 * Declares the API for robot entity management.
 * Robots are the active agents of the simulation: they move between shelves,
 * packing stations, and charging stations in response to scheduled events.
 *
 * NOTE: destroy_robot does NOT free current_order. The order's lifetime is
 * managed externally by the event system and the pending queue.
 *
 * Depends on: types.h
 */

#include "types.h"

/*
 * create_robot
 * ------------
 * Heap-allocates and initialises a Robot with full charge and IDLE state.
 *
 * Parameters:
 *   idx       - 0-based index; robot ID is formatted as "R%03d" (idx+1),
 *               e.g. idx=0 → "R001", idx=19 → "R020"
 *   start_pos - initial grid position (should be a walkable tile)
 *
 * Returns:
 *   Robot* with battery=100, state=ROBOT_IDLE, all counters zeroed,
 *   current_order=NULL. Exits on allocation failure.
 */
Robot *create_robot(int idx, Position start_pos);

/*
 * destroy_robot
 * -------------
 * Frees the Robot struct.
 * IMPORTANT: current_order is intentionally NOT freed here because order
 * ownership belongs to the event system. Always call destroy_order on any
 * in-progress order BEFORE calling destroy_robot during cleanup.
 *
 * Parameters:
 *   robot - Robot to free; no-op if NULL.
 */
void destroy_robot(Robot *robot);

/*
 * print_robot_status
 * ------------------
 * Prints a compact one-line status summary to stdout.
 * Format: "  Robot RXXX | Pos: [xx,yy] | Battery: XXX% | State: STATE | Orders: N"
 *
 * Parameters:
 *   robot - valid, non-NULL Robot*
 */
void print_robot_status(const Robot *robot);

/*
 * find_idle_robot
 * ---------------
 * Linear scan of the robots array for the first ROBOT_IDLE entry.
 *
 * Parameters:
 *   robots - array of Robot* pointers (may contain NULL entries)
 *   n      - length of the array
 *
 * Returns:
 *   Index of the first idle robot; -1 if no robot is idle.
 */
int find_idle_robot(Robot **robots, int n);

#endif /* ROBOT_H */
