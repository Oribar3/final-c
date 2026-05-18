#ifndef EVENT_H
#define EVENT_H

/*
 * event.h
 * -------
 * Declares the Future Event List (FEL) API and all four discrete event
 * handler functions that implement the core simulation logic.
 *
 * The FEL is a singly-linked list kept in strict ascending timestamp order.
 * O(N) sorted insertion ensures the next event is always at the head,
 * enabling O(1) extraction of the minimum-time event.
 *
 * Depends on: types.h, warehouse.h, pathfinding.h, robot.h, order.h, stats.h
 */

#include "types.h"
#include "warehouse.h"
#include "pathfinding.h"
#include "robot.h"
#include "order.h"
#include "stats.h"

/* ============================================================
 * Future Event List Operations
 * ============================================================ */

/*
 * create_fel
 * ----------
 * Allocates an empty FEL (head=NULL, count=0).
 *
 * Returns: FEL*; exits on allocation failure.
 */
FEL *create_fel(void);

/*
 * destroy_fel
 * -----------
 * Frees every EventNode in the list, then the FEL struct itself.
 *
 * Parameters:
 *   fel - may be NULL (no-op).
 */
void destroy_fel(FEL *fel);

/*
 * insert_event_sorted
 * -------------------
 * Wraps data in a new heap-allocated EventNode and inserts it into the FEL
 * so that the list remains sorted in ascending order of EventData.time.
 * Ties (equal timestamps) are broken in insertion order (stable).
 * Complexity: O(N) in the number of events currently scheduled.
 *
 * Parameters:
 *   fel  - valid, non-NULL FEL*
 *   data - EventData value to schedule
 *
 * Side effects: allocates one EventNode; exits on failure.
 */
void insert_event_sorted(FEL *fel, EventData data);

/*
 * pop_next_event
 * --------------
 * Removes and returns the head EventNode (earliest scheduled time).
 * The caller is responsible for calling free() on the returned node
 * after extracting its data field.
 *
 * Parameters:
 *   fel - valid FEL*
 *
 * Returns: EventNode* (caller owns it); NULL if FEL is empty.
 */
EventNode *pop_next_event(FEL *fel);

/*
 * is_fel_empty
 * ------------
 * Returns 1 if the FEL contains no events, 0 otherwise.
 */
int is_fel_empty(const FEL *fel);

/*
 * peek_next_time
 * --------------
 * Returns the scheduled time of the next event without removing it.
 *
 * Returns: time of the head event; -1.0 if the FEL is empty.
 */
double peek_next_time(const FEL *fel);

/* ============================================================
 * Event Handlers
 * ============================================================ */

/*
 * handle_order_arrival
 * --------------------
 * Processes EVENT_ORDER_ARRIVAL:
 *   1. Picks a random TILE_SHELF position and creates a new Order.
 *   2. Assigns the order to an idle robot (if one exists) and schedules
 *      EVENT_ROBOT_REACHED_SHELF, OR enqueues the order if all robots are busy.
 *   3. Schedules the next ORDER_ARRIVAL at an exponentially-distributed
 *      future time.
 *
 * Parameters:
 *   ctx - full simulation context (mutated: FEL, pending queue, stats)
 *   ev  - the triggering event (ev.time == ctx->current_time at call site)
 */
void handle_order_arrival(SimContext *ctx, EventData ev);

/*
 * handle_robot_reached_shelf
 * --------------------------
 * Processes EVENT_ROBOT_REACHED_SHELF:
 *   1. Moves the robot to the order's shelf position.
 *   2. Drains battery proportional to the BFS distance travelled.
 *   3. Finds the nearest packing station via BFS and schedules
 *      EVENT_ROBOT_REACHED_PACKING.
 *
 * Parameters:
 *   ctx - simulation context
 *   ev  - event data (ev.robot_idx identifies the robot)
 */
void handle_robot_reached_shelf(SimContext *ctx, EventData ev);

/*
 * handle_robot_reached_packing
 * ----------------------------
 * Processes EVENT_ROBOT_REACHED_PACKING:
 *   1. Moves the robot to the packing station and drains its battery.
 *   2. Marks the order as COMPLETED and updates statistics.
 *   3. Decides the robot's next action:
 *        - Battery <= BATTERY_LOW_THRESH → schedule charging.
 *        - Pending order available       → assign it.
 *        - Otherwise                     → set robot to IDLE.
 *
 * Parameters:
 *   ctx - simulation context
 *   ev  - event data (ev.robot_idx identifies the robot)
 */
void handle_robot_reached_packing(SimContext *ctx, EventData ev);

/*
 * handle_robot_charging_complete
 * ------------------------------
 * Processes EVENT_ROBOT_CHARGING_COMPLETE:
 *   1. Moves the robot to the charging station and restores battery to 100%.
 *   2. Increments the robot's charging cycle counter.
 *   3. Assigns a pending order if one exists, otherwise sets robot to IDLE.
 *
 * Parameters:
 *   ctx - simulation context
 *   ev  - event data (ev.robot_idx identifies the robot)
 */
void handle_robot_charging_complete(SimContext *ctx, EventData ev);

#endif /* EVENT_H */
