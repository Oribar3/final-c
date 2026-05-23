#define _CRT_SECURE_NO_WARNINGS

/*
 * robot.c
 * -------
 * Implements robot entity creation, destruction, status display, and
 * the idle-robot search used by the event system to dispatch orders.
 */

#include "../include/robot.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ============================================================
 * Internal Helper
 * ============================================================ */

/*
 * robot_state_name
 * ----------------
 * Converts a RobotState enum value to a human-readable string.
 * A switch (rather than a lookup table) makes adding new states safe.
 *
 * Parameters:
 *   s - RobotState enum value
 *
 * Returns: static string; caller must NOT free it.
 */
static const char *robot_state_name(RobotState s)
{
    switch (s) {
        case ROBOT_IDLE:              return "IDLE";
        case ROBOT_MOVING_TO_SHELF:   return "MOVING_TO_SHELF";
        case ROBOT_MOVING_TO_PACKING: return "MOVING_TO_PACKING";
        case ROBOT_CHARGING:          return "CHARGING";
        default:                      return "UNKNOWN";
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

Robot *create_robot(int idx, Position start_pos)
{
    Robot *r = (Robot *)malloc(sizeof(Robot));
    if (!r) {
        fprintf(stderr, "ERROR: Failed to allocate Robot.\n");
        exit(EXIT_FAILURE);
    }
    /* 1-based ID: idx=0 → "R001" */
    snprintf(r->id, MAX_ID_LEN, "R%03d", idx + 1);
    r->pos              = start_pos;
    r->battery          = 100;
    r->state            = ROBOT_IDLE;
    snprintf(r->status_desc, MAX_STATUS_LEN, "Initialised at [%d,%d]",
             start_pos.x, start_pos.y);
    r->orders_completed = 0;
    r->charging_cycles  = 0;
    r->total_busy_time  = 0.0;
    r->busy_start_time  = 0.0;
    r->current_order    = NULL; /* Managed externally; never freed by destroy_robot */
    return r;
}

void destroy_robot(Robot *robot)
{
    if (!robot) return;
    /* current_order is intentionally NOT freed here — see header note */
    free(robot);
}

void print_robot_status(const Robot *robot)
{
    printf("  Robot %-6s | Pos: [%2d,%2d] | Battery: %3d%% | "
           "State: %-22s | Orders: %d\n",
           robot->id,
           robot->pos.x, robot->pos.y,
           robot->battery,
           robot_state_name(robot->state),
           robot->orders_completed);
}

int find_idle_robot(Robot **robots, int n)
{
    int i;
    for (i = 0; i < n; i++) {
        if (robots[i] && robots[i]->state == ROBOT_IDLE) return i;
    }
    return -1;
}
