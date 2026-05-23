#define _CRT_SECURE_NO_WARNINGS

/*
 * event.c
 * -------
 * Implements the Future Event List and all four discrete event handlers.
 *
 * The FEL is a singly-linked list sorted ascending by event time.
 * insert_event_sorted walks the list once (O(N)) to find the insertion
 * point, keeping the head always the next event to process.
 *
 * Event handlers share the static helper assign_order_to_robot so that
 * the pattern "pick an idle robot, compute BFS distance, schedule arrival"
 * is never duplicated across handle_order_arrival, handle_robot_reached_packing,
 * and handle_robot_charging_complete.
 */

#include "../include/event.h"
#include "../include/fileio.h"    /* save_event_log_entry */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ============================================================
 * FEL Implementation
 * ============================================================ */

FEL *create_fel(void)
{
    FEL *fel = (FEL *)malloc(sizeof(FEL));
    if (!fel) {
        fprintf(stderr, "ERROR: Failed to allocate FEL.\n");
        exit(EXIT_FAILURE);
    }
    fel->head  = NULL;
    fel->count = 0;
    return fel;
}

void destroy_fel(FEL *fel)
{
    if (!fel) return;
    EventNode *cur = fel->head;
    while (cur) {
        EventNode *next = cur->next;
        free(cur);
        cur = next;
    }
    free(fel);
}

void insert_event_sorted(FEL *fel, EventData data)
{
    EventNode *node = (EventNode *)malloc(sizeof(EventNode));
    if (!node) {
        fprintf(stderr, "ERROR: Failed to allocate EventNode.\n");
        exit(EXIT_FAILURE);
    }
    node->data = data;
    node->next = NULL;

    /* Case 1: empty list, or new event is earlier than current head */
    if (!fel->head || data.time < fel->head->data.time) {
        node->next = fel->head;
        fel->head  = node;
        fel->count++;
        return;
    }

    /* Case 2: find the last node whose time <= new event time (stable sort) */
    EventNode *cur = fel->head;
    while (cur->next && cur->next->data.time <= data.time) {
        cur = cur->next;
    }
    node->next = cur->next;
    cur->next  = node;
    fel->count++;
}

EventNode *pop_next_event(FEL *fel)
{
    if (!fel->head) return NULL;
    EventNode *node = fel->head;
    fel->head       = node->next;
    node->next      = NULL; /* Caller receives a clean node */
    fel->count--;
    return node;
}

int is_fel_empty(const FEL *fel)
{
    return fel->count == 0;
}

double peek_next_time(const FEL *fel)
{
    if (!fel->head) return -1.0;
    return fel->head->data.time;
}

/* ============================================================
 * Internal Static Helpers
 * ============================================================ */

/*
 * exponential_random
 * ------------------
 * Generates a random inter-arrival delay from an exponential distribution
 * with mean avg_interval, using the inverse-CDF method: -mean * ln(U).
 * The +1e-10 epsilon prevents undefined log(0) when rand() returns 0.
 *
 * Parameters:
 *   avg_interval - mean of the distribution (must be > 0)
 *
 * Returns: positive double delay value.
 */
static double exponential_random(double avg_interval)
{
    double u = (double)rand() / (double)RAND_MAX + 1e-10;
    return -avg_interval * log(u);
}

/*
 * find_random_shelf
 * -----------------
 * Scans the entire warehouse grid for TILE_SHELF tiles, collects their
 * positions into a temporary heap array, and returns one chosen uniformly
 * at random. This ensures orders are distributed across all shelves.
 *
 * Parameters:
 *   wh - valid Warehouse* (read-only)
 *
 * Returns: a random shelf Position; {-1,-1} if no shelves exist.
 */
static Position find_random_shelf(const Warehouse *wh)
{
    Position not_found;
    not_found.x = -1;
    not_found.y = -1;

    /* First pass: count shelves */
    int count = 0;
    int x, y;
    for (y = 0; y < wh->height; y++)
        for (x = 0; x < wh->width; x++)
            if (get_tile(wh, x, y) == TILE_SHELF) count++;

    if (count == 0) return not_found;

    /* Second pass: collect shelf positions */
    Position *shelves = (Position *)malloc((size_t)count * sizeof(Position));
    if (!shelves) {
        fprintf(stderr, "ERROR: find_random_shelf: allocation failed.\n");
        return not_found;
    }

    int idx = 0;
    for (y = 0; y < wh->height; y++) {
        for (x = 0; x < wh->width; x++) {
            if (get_tile(wh, x, y) == TILE_SHELF) {
                shelves[idx].x = x;
                shelves[idx].y = y;
                idx++;
            }
        }
    }

    Position chosen = shelves[rand() % count];
    free(shelves);
    return chosen;
}

/*
 * assign_order_to_robot
 * ----------------------
 * Sets a robot's state to MOVING_TO_SHELF, links it to the given order,
 * computes the BFS travel distance from the robot's current position to
 * the shelf, and schedules the EVENT_ROBOT_REACHED_SHELF event.
 *
 * This function is the single point of truth for order assignment so that
 * handle_order_arrival, handle_robot_reached_packing, and
 * handle_robot_charging_complete all use identical logic.
 *
 * Parameters:
 *   ctx       - simulation context
 *   robot_idx - index into ctx->robots
 *   order     - Order to assign; state is set to IN_PROGRESS inside
 */
static void assign_order_to_robot(SimContext *ctx, int robot_idx, Order *order)
{
    Robot *r = ctx->robots[robot_idx];

    order->state     = ORDER_IN_PROGRESS;
    r->current_order = order;
    r->state         = ROBOT_MOVING_TO_SHELF;
    r->busy_start_time = ctx->current_time;
    snprintf(r->status_desc, MAX_STATUS_LEN, "Heading to shelf [%d,%d]",
             order->shelf_pos.x, order->shelf_pos.y);

    int dist = bfs_distance(ctx->warehouse, r->pos, order->shelf_pos);
    if (dist < 0) dist = 1; /* Fallback: treat as adjacent if BFS fails */

    EventData ev;
    ev.type      = EVENT_ROBOT_REACHED_SHELF;
    ev.time      = ctx->current_time + (double)dist * TIME_PER_STEP;
    ev.robot_idx = robot_idx;
    ev.order_id  = -1;
    insert_event_sorted(ctx->fel, ev);

    save_event_log_entry(ctx->log_file,
        "T=%7.2f: Robot %s assigned Order %s -> Shelf [%d,%d] "
        "(travel dist=%d, ETA=%.2f)\n",
        ctx->current_time, r->id, order->id,
        order->shelf_pos.x, order->shelf_pos.y,
        dist, ev.time);
}

/* ============================================================
 * Event Handlers
 * ============================================================ */

void handle_order_arrival(SimContext *ctx, EventData ev)
{
    (void)ev; /* ev.time already captured in ctx->current_time */

    Position shelf = find_random_shelf(ctx->warehouse);
    if (shelf.x == -1) {
        fprintf(stderr,
            "WARNING T=%.2f: No shelf tiles in warehouse. Order skipped.\n",
            ctx->current_time);
    } else {
        Order *order = create_order(ctx->next_order_id++, shelf,
                                    ctx->current_time);
        ctx->stats->total_orders_arrived++;

        save_event_log_entry(ctx->log_file,
            "T=%7.2f: [ORDER ARRIVAL] Order %s arrived. "
            "Target shelf: [%d,%d]\n",
            ctx->current_time, order->id, shelf.x, shelf.y);

        int idle = find_idle_robot(ctx->robots, ctx->num_robots);
        if (idle >= 0) {
            assign_order_to_robot(ctx, idle, order);
        } else {
            enqueue_order(ctx->pending_orders, order);
            update_queue_length(ctx->stats, ctx->pending_orders->count);
            save_event_log_entry(ctx->log_file,
                "T=%7.2f: No idle robot available. Order %s queued "
                "(queue length: %d)\n",
                ctx->current_time, order->id,
                ctx->pending_orders->count);
        }
    }

    /* Schedule the next order arrival using exponential inter-arrival time */
    double next_time = ctx->current_time +
                       exponential_random(ctx->avg_order_interval);
    if (next_time <= ctx->time_limit) {
        EventData next;
        next.type      = EVENT_ORDER_ARRIVAL;
        next.time      = next_time;
        next.robot_idx = -1;
        next.order_id  = -1;
        insert_event_sorted(ctx->fel, next);
    }
}

void handle_robot_reached_shelf(SimContext *ctx, EventData ev)
{
    Robot   *r       = ctx->robots[ev.robot_idx];
    Order   *order   = r->current_order;
    Position old_pos = r->pos;

    /* Move robot to the shelf position */
    r->pos = order->shelf_pos;

    /* Drain battery for the distance travelled to reach the shelf */
    int dist_to_shelf = bfs_distance(ctx->warehouse, old_pos, r->pos);
    if (dist_to_shelf < 0) dist_to_shelf = 1;
    r->battery -= dist_to_shelf * BATTERY_PER_STEP;
    if (r->battery < 0) r->battery = 0;

    r->state = ROBOT_MOVING_TO_PACKING;
    snprintf(r->status_desc, MAX_STATUS_LEN,
             "At shelf [%d,%d], heading to packing", r->pos.x, r->pos.y);

    save_event_log_entry(ctx->log_file,
        "T=%7.2f: Robot %s reached Shelf [%d,%d]. Battery: %d%%\n",
        ctx->current_time, r->id, r->pos.x, r->pos.y, r->battery);

    /* Locate the nearest packing station from the shelf */
    Position packing = find_nearest_tile_bfs(ctx->warehouse, r->pos,
                                             TILE_PACKING);
    if (packing.x == -1) {
        fprintf(stderr,
            "ERROR T=%.2f: Robot %s cannot find a packing station! "
            "Check warehouse layout.\n",
            ctx->current_time, r->id);
        r->state = ROBOT_IDLE;
        return;
    }

    int dist_to_packing = bfs_distance(ctx->warehouse, r->pos, packing);
    if (dist_to_packing < 0) dist_to_packing = 1;

    EventData next;
    next.type      = EVENT_ROBOT_REACHED_PACKING;
    next.time      = ctx->current_time +
                     (double)dist_to_packing * TIME_PER_STEP;
    next.robot_idx = ev.robot_idx;
    next.order_id  = -1;
    insert_event_sorted(ctx->fel, next);
}

void handle_robot_reached_packing(SimContext *ctx, EventData ev)
{
    Robot   *r         = ctx->robots[ev.robot_idx];
    Order   *order     = r->current_order;
    Position shelf_pos = r->pos; /* r->pos is still the shelf at this point */

    /* Re-derive the packing station (deterministic BFS from shelf pos) */
    Position packing = find_nearest_tile_bfs(ctx->warehouse, shelf_pos,
                                             TILE_PACKING);
    if (packing.x == -1) {
        fprintf(stderr,
            "ERROR T=%.2f: Robot %s: no packing station reachable.\n",
            ctx->current_time, r->id);
        r->state = ROBOT_IDLE;
        destroy_order(order);
        r->current_order = NULL;
        return;
    }

    /* Drain battery for the shelf-to-packing leg */
    int dist = bfs_distance(ctx->warehouse, shelf_pos, packing);
    if (dist < 0) dist = 1;
    r->battery -= dist * BATTERY_PER_STEP;
    if (r->battery < 0) r->battery = 0;

    /* Move robot to the packing station */
    r->pos = packing;

    /* Mark order as completed and record timing */
    order->state           = ORDER_COMPLETED;
    order->completion_time = ctx->current_time;
    double waiting_time    = ctx->current_time - order->arrival_time;

    update_order_completed(ctx->stats, waiting_time);
    r->orders_completed++;

    /* Accumulate robot busy time for utilization metric */
    r->total_busy_time += ctx->current_time - r->busy_start_time;

    save_event_log_entry(ctx->log_file,
        "T=%7.2f: Robot %s reached Packing [%d,%d]. "
        "Order %s COMPLETED. Wait: %.2f units | Battery: %d%%\n",
        ctx->current_time, r->id, r->pos.x, r->pos.y,
        order->id, waiting_time, r->battery);

    /* Release the completed order */
    destroy_order(order);
    r->current_order = NULL;

    /* Decide robot's next action */
    if (r->battery <= BATTERY_LOW_THRESH) {
        /* Battery is low — route to nearest charging station */
        Position charger = find_nearest_tile_bfs(ctx->warehouse, r->pos,
                                                  TILE_CHARGING);
        if (charger.x == -1) {
            fprintf(stderr,
                "WARNING T=%.2f: Robot %s: no charging station found. "
                "Setting to IDLE.\n",
                ctx->current_time, r->id);
            r->state = ROBOT_IDLE;
            return;
        }

        int dist_c = bfs_distance(ctx->warehouse, r->pos, charger);
        if (dist_c < 0) dist_c = 1;

        r->state = ROBOT_CHARGING;
        snprintf(r->status_desc, MAX_STATUS_LEN,
                 "Battery %d%% — routing to charger [%d,%d]",
                 r->battery, charger.x, charger.y);

        save_event_log_entry(ctx->log_file,
            "T=%7.2f: Robot %s battery LOW (%d%%). "
            "Routing to Charger [%d,%d] (ETA %.2f).\n",
            ctx->current_time, r->id, r->battery, charger.x, charger.y,
            ctx->current_time + (double)dist_c * TIME_PER_STEP + CHARGE_DURATION);

        EventData charge_ev;
        charge_ev.type      = EVENT_ROBOT_CHARGING_COMPLETE;
        charge_ev.time      = ctx->current_time +
                              (double)dist_c * TIME_PER_STEP + CHARGE_DURATION;
        charge_ev.robot_idx = ev.robot_idx;
        charge_ev.order_id  = -1;
        insert_event_sorted(ctx->fel, charge_ev);

    } else if (!is_queue_empty(ctx->pending_orders)) {
        /* Battery is fine — pick up the next pending order */
        Order *pending = dequeue_order(ctx->pending_orders);
        update_queue_length(ctx->stats, ctx->pending_orders->count);
        assign_order_to_robot(ctx, ev.robot_idx, pending);

    } else {
        /* No work and sufficient charge — go idle */
        r->state = ROBOT_IDLE;
        snprintf(r->status_desc, MAX_STATUS_LEN,
                 "Idle at packing station [%d,%d]", r->pos.x, r->pos.y);
        save_event_log_entry(ctx->log_file,
            "T=%7.2f: Robot %s is now IDLE at [%d,%d].\n",
            ctx->current_time, r->id, r->pos.x, r->pos.y);
    }
}

void handle_robot_charging_complete(SimContext *ctx, EventData ev)
{
    Robot *r = ctx->robots[ev.robot_idx];

    /*
     * r->pos is currently the packing station (the last position updated in
     * handle_robot_reached_packing). Re-derive the charger via the same
     * deterministic BFS so we move the robot to the correct tile.
     */
    Position charger = find_nearest_tile_bfs(ctx->warehouse, r->pos,
                                              TILE_CHARGING);
    if (charger.x != -1) {
        r->pos = charger;
    }

    r->battery = 100;
    r->charging_cycles++;
    snprintf(r->status_desc, MAX_STATUS_LEN,
             "Fully charged at [%d,%d]", r->pos.x, r->pos.y);

    save_event_log_entry(ctx->log_file,
        "T=%7.2f: Robot %s fully charged at [%d,%d]. Battery: 100%%\n",
        ctx->current_time, r->id, r->pos.x, r->pos.y);

    /* Assign a waiting order or go idle */
    if (!is_queue_empty(ctx->pending_orders)) {
        Order *pending = dequeue_order(ctx->pending_orders);
        update_queue_length(ctx->stats, ctx->pending_orders->count);
        assign_order_to_robot(ctx, ev.robot_idx, pending);
    } else {
        r->state = ROBOT_IDLE;
        snprintf(r->status_desc, MAX_STATUS_LEN,
                 "Idle at charger [%d,%d]", r->pos.x, r->pos.y);
        save_event_log_entry(ctx->log_file,
            "T=%7.2f: Robot %s is now IDLE at charger [%d,%d].\n",
            ctx->current_time, r->id, r->pos.x, r->pos.y);
    }
}
