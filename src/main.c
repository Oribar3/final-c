#define _CRT_SECURE_NO_WARNINGS

/*
 * main.c
 * ------
 * Entry point and top-level orchestration for the Automated Robotic Warehouse
 * Discrete-Event Simulation.
 *
 * Responsibilities:
 *   - Present an interactive console menu.
 *   - Accept and validate simulation parameters from the user.
 *   - Build a minimal warehouse programmatically for manual-parameter runs.
 *   - Execute the DES main loop (pop → dispatch → repeat).
 *   - Call every destroy_* function to prevent memory leaks on exit.
 *
 * All simulation logic lives in the event handlers (event.c).
 * This file contains only orchestration and I/O.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "types.h"
#include "warehouse.h"
#include "pathfinding.h"
#include "order.h"
#include "robot.h"
#include "stats.h"
#include "event.h"
#include "fileio.h"

/* ============================================================
 * Forward Declarations
 * ============================================================ */
static void        show_menu(void);
static SimContext *get_simulation_params_interactive(void);
static void        run_simulation(SimContext *ctx);
static void        dispatch_event(SimContext *ctx, EventData ev);
static void        cleanup_simulation(SimContext *ctx);

/* ============================================================
 * Console I/O Helpers
 * ============================================================ */

/*
 * flush_input_buffer
 * ------------------
 * Discards all characters remaining in stdin up to and including the next
 * newline. Called after every scanf to prevent stale input from affecting
 * subsequent reads.
 */
static void flush_input_buffer(void)
{
    int c;
    while ((c = getchar()) != '\n' && c != EOF)
        ;
}

/*
 * read_int_in_range
 * -----------------
 * Prompts the user and reads an integer, re-prompting on invalid input or
 * out-of-range values.
 *
 * Parameters:
 *   prompt  - string printed before each read attempt
 *   min_val - minimum acceptable value (inclusive)
 *   max_val - maximum acceptable value (inclusive)
 *
 * Returns: validated integer in [min_val, max_val].
 */
static int read_int_in_range(const char *prompt, int min_val, int max_val)
{
    int val;
    while (1) {
        printf("%s", prompt);
        if (scanf("%d", &val) == 1 && val >= min_val && val <= max_val) {
            flush_input_buffer();
            return val;
        }
        fprintf(stderr,
            "  Invalid input. Please enter an integer between %d and %d.\n",
            min_val, max_val);
        flush_input_buffer();
    }
}

/*
 * read_double_positive
 * --------------------
 * Prompts the user and reads a positive double, re-prompting on failure.
 *
 * Parameters:
 *   prompt - string printed before each read attempt
 *
 * Returns: validated positive double.
 */
static double read_double_positive(const char *prompt)
{
    double val;
    while (1) {
        printf("%s", prompt);
        if (scanf("%lf", &val) == 1 && val > 0.0) {
            flush_input_buffer();
            return val;
        }
        fprintf(stderr, "  Invalid input. Please enter a positive number.\n");
        flush_input_buffer();
    }
}

/*
 * print_separator
 * ---------------
 * Prints a decorative line to visually separate sections of output.
 */
static void print_separator(void)
{
    printf("----------------------------------------------------\n");
}

/* ============================================================
 * Menu
 * ============================================================ */

static void show_menu(void)
{
    printf("\n");
    printf("====================================================\n");
    printf("   Automated Robotic Warehouse Simulation  (DES)\n");
    printf("====================================================\n");
    printf("  1. Run simulation with manual parameters\n");
    printf("  2. Load configuration from '%s'\n", DEFAULT_MAP_FILE);
    printf("  3. Generate default configuration file\n");
    printf("  4. Exit\n");
    print_separator();
    printf("  Your choice: ");
}

/* ============================================================
 * Interactive Parameter Input & Warehouse Construction
 * ============================================================ */

/*
 * build_interactive_warehouse
 * ---------------------------
 * Constructs a warehouse grid from user-supplied dimensions with a
 * programmatic layout:
 *   - Full border of TILE_WALL
 *   - Interior filled with TILE_FLOOR
 *   - Shelf at the grid centre
 *   - Packing station at the top-right interior corner
 *   - Charging station at the bottom-left interior corner
 *   - Extra shelves, packing, and charging tiles for larger maps
 *
 * Parameters:
 *   width  - warehouse width  (>= 5)
 *   height - warehouse height (>= 5)
 *
 * Returns: fully configured Warehouse*.
 */
static Warehouse *build_interactive_warehouse(int width, int height)
{
    Warehouse *wh = create_warehouse(width, height);
    int x, y;

    /* Draw wall border */
    for (x = 0; x < width; x++) {
        set_tile(wh, x, 0,        TILE_WALL);
        set_tile(wh, x, height-1, TILE_WALL);
    }
    for (y = 1; y < height - 1; y++) {
        set_tile(wh, 0,       y, TILE_WALL);
        set_tile(wh, width-1, y, TILE_WALL);
    }
    /* All interior tiles are TILE_FLOOR by default (calloc zeroed them) */

    int cx = width  / 2;  /* Centre column */
    int cy = height / 2;  /* Centre row    */

    /* Mandatory tiles — always placed in valid interior positions */
    set_tile(wh, cx, cy,         TILE_SHELF);    /* Shelf at centre          */
    set_tile(wh, width-2, 1,     TILE_PACKING);  /* Packing: top-right       */
    set_tile(wh, 1, height-2,    TILE_CHARGING); /* Charging: bottom-left    */

    /* Extra shelves along the centre row for larger maps */
    if (width >= 7) {
        set_tile(wh, 2,       cy, TILE_SHELF);
        set_tile(wh, width-3, cy, TILE_SHELF);
    }
    /* Extra packing and charging for taller maps */
    if (height >= 7) {
        set_tile(wh, width-2, height-2, TILE_PACKING);
        set_tile(wh, 1,       1,        TILE_CHARGING);
    }

    return wh;
}

/*
 * get_simulation_params_interactive
 * ----------------------------------
 * Prompts the user for all simulation parameters, builds a warehouse, places
 * robots, and returns a fully initialised SimContext.
 *
 * Returns: SimContext* ready for run_simulation. Exits on fatal failure.
 */
static SimContext *get_simulation_params_interactive(void)
{
    printf("\n  --- Manual Simulation Setup ---\n");

    int    width    = read_int_in_range("  Warehouse width  (5-50): ", 5, 50);
    int    height   = read_int_in_range("  Warehouse height (5-50): ", 5, 50);
    int    nrobots  = read_int_in_range("  Number of robots (1-20): ", 1, MAX_ROBOTS);
    double tlimit   = read_double_positive("  Simulation time limit   (e.g. 100.0): ");
    double avgint   = read_double_positive("  Avg order arrival interval (e.g. 5.0): ");

    Warehouse *wh = build_interactive_warehouse(width, height);

    /* Place robots on the first nrobots TILE_FLOOR tiles (row-major scan) */
    Robot **robots = (Robot **)calloc((size_t)nrobots, sizeof(Robot *));
    if (!robots) {
        fprintf(stderr, "ERROR: Failed to allocate robots array.\n");
        destroy_warehouse(wh);
        exit(EXIT_FAILURE);
    }

    int placed = 0, x, y;
    for (y = 0; y < height && placed < nrobots; y++) {
        for (x = 0; x < width && placed < nrobots; x++) {
            if (get_tile(wh, x, y) == TILE_FLOOR) {
                Position p;
                p.x = x;
                p.y = y;
                robots[placed] = create_robot(placed, p);
                placed++;
            }
        }
    }

    if (placed == 0) {
        fprintf(stderr,
            "ERROR: No floor tiles available for robot placement.\n");
        free(robots);
        destroy_warehouse(wh);
        exit(EXIT_FAILURE);
    }
    if (placed < nrobots) {
        fprintf(stderr,
            "WARNING: Only %d floor tiles found; using %d robots.\n",
            placed, placed);
        nrobots = placed;
    }

    SimContext *ctx = (SimContext *)malloc(sizeof(SimContext));
    if (!ctx) {
        fprintf(stderr, "ERROR: Failed to allocate SimContext.\n");
        int i;
        for (i = 0; i < placed; i++) destroy_robot(robots[i]);
        free(robots);
        destroy_warehouse(wh);
        exit(EXIT_FAILURE);
    }

    ctx->warehouse          = wh;
    ctx->robots             = robots;
    ctx->num_robots         = nrobots;
    ctx->pending_orders     = create_order_queue();
    ctx->fel                = create_fel();
    ctx->stats              = create_stats();
    ctx->current_time       = 0.0;
    ctx->time_limit         = tlimit;
    ctx->avg_order_interval = avgint;
    ctx->next_order_id      = 1;
    ctx->next_event_id      = 1;
    ctx->log_file           = NULL;

    return ctx;
}

/* ============================================================
 * Simulation Loop
 * ============================================================ */

/*
 * dispatch_event
 * --------------
 * Routes a single dequeued event to the appropriate handler function
 * based on its EventType. Increments next_event_id for logging.
 *
 * Parameters:
 *   ctx - simulation context (passed through to handlers)
 *   ev  - event data extracted from the popped FEL node
 */
static void dispatch_event(SimContext *ctx, EventData ev)
{
    ctx->next_event_id++;
    switch (ev.type) {
        case EVENT_ORDER_ARRIVAL:
            handle_order_arrival(ctx, ev);
            break;
        case EVENT_ROBOT_REACHED_SHELF:
            handle_robot_reached_shelf(ctx, ev);
            break;
        case EVENT_ROBOT_REACHED_PACKING:
            handle_robot_reached_packing(ctx, ev);
            break;
        case EVENT_ROBOT_CHARGING_COMPLETE:
            handle_robot_charging_complete(ctx, ev);
            break;
        default:
            fprintf(stderr,
                "WARNING: Unknown event type %d at T=%.2f — skipped.\n",
                ev.type, ev.time);
            break;
    }
}

/*
 * run_simulation
 * --------------
 * Executes the discrete-event simulation main loop:
 *   1. Opens the event log file.
 *   2. Primes the FEL with an ORDER_ARRIVAL at T=0.
 *   3. Repeatedly pops the earliest event, advances the clock, dispatches.
 *   4. Terminates when the FEL is empty or the time limit is exceeded.
 *   5. Generates the final analytical report.
 *
 * Parameters:
 *   ctx - fully initialised SimContext*
 */
static void run_simulation(SimContext *ctx)
{
    printf("\n");
    print_separator();
    printf("  SIMULATION START  (time limit: %.2f time units)\n",
           ctx->time_limit);
    print_separator();

    print_warehouse(ctx->warehouse);

    printf("  Initial Robot Positions:\n");
    int i;
    for (i = 0; i < ctx->num_robots; i++) {
        print_robot_status(ctx->robots[i]);
    }
    printf("\n");
    print_separator();
    printf("  SIMULATION EVENT LOG\n");
    print_separator();

    /* Open the persistent event log */
    ctx->log_file = open_log_file(DEFAULT_LOG_FILE);

    /* Prime the FEL: first order arrival at T=0 */
    EventData first;
    first.type      = EVENT_ORDER_ARRIVAL;
    first.time      = 0.0;
    first.robot_idx = -1;
    first.order_id  = -1;
    insert_event_sorted(ctx->fel, first);

    /* ---- Main DES loop ---- */
    while (!is_fel_empty(ctx->fel)) {
        EventNode *node = pop_next_event(ctx->fel);
        ctx->current_time = node->data.time;

        /* Stop processing once the clock passes the time limit */
        if (ctx->current_time > ctx->time_limit) {
            free(node);
            break;
        }

        dispatch_event(ctx, node->data);
        free(node); /* EventNode ownership transferred by pop_next_event */
    }

    /* ---- Simulation ended ---- */
    printf("\n");
    print_separator();
    printf("  SIMULATION ENDED  (clock: T=%.2f)\n", ctx->current_time);
    print_separator();

    printf("  Final Robot Status:\n");
    for (i = 0; i < ctx->num_robots; i++) {
        print_robot_status(ctx->robots[i]);
    }

    /* Save report to file, then print to console (also mirrors to log) */
    save_report_to_file(ctx->stats, ctx->robots, ctx->num_robots,
                        ctx->current_time, DEFAULT_REPORT_FILE);
    print_final_report(ctx->stats, ctx->robots, ctx->num_robots,
                       ctx->current_time, ctx->log_file);

    close_log_file(ctx->log_file);
    ctx->log_file = NULL;

    printf("  Event log saved to '%s'.\n\n", DEFAULT_LOG_FILE);
}

/* ============================================================
 * Cleanup
 * ============================================================ */

/*
 * cleanup_simulation
 * ------------------
 * Frees every heap allocation owned by the simulation context in the correct
 * order to prevent memory leaks or use-after-free errors:
 *
 *   1. Any order a robot is still holding (freed BEFORE destroy_robot, which
 *      does NOT free current_order to avoid double-free on completed orders).
 *   2. Each Robot*, then the pointer array.
 *   3. The pending-order queue (frees remaining order nodes within it).
 *   4. Remaining FEL nodes.
 *   5. The warehouse grid (row-by-row) and struct.
 *   6. The statistics struct.
 *   7. The log file (if somehow still open).
 *   8. The SimContext struct itself.
 *
 * Parameters:
 *   ctx - SimContext to free; no-op if NULL.
 */
static void cleanup_simulation(SimContext *ctx)
{
    if (!ctx) return;

    int i;

    /* Step 1: Free any order a robot is still carrying */
    for (i = 0; i < ctx->num_robots; i++) {
        if (ctx->robots[i] && ctx->robots[i]->current_order) {
            destroy_order(ctx->robots[i]->current_order);
            ctx->robots[i]->current_order = NULL;
        }
    }

    /* Step 2: Free each robot, then the pointer array */
    for (i = 0; i < ctx->num_robots; i++) {
        destroy_robot(ctx->robots[i]);
    }
    free(ctx->robots);

    /* Step 3: Free the pending queue (and all remaining order nodes) */
    destroy_order_queue(ctx->pending_orders);

    /* Step 4: Free remaining FEL nodes */
    destroy_fel(ctx->fel);

    /* Step 5: Free the warehouse */
    destroy_warehouse(ctx->warehouse);

    /* Step 6: Free statistics */
    destroy_stats(ctx->stats);

    /* Step 7: Close log file if still open (e.g. error path) */
    close_log_file(ctx->log_file);

    /* Step 8: Free the context struct */
    free(ctx);
}

/* ============================================================
 * Entry Point
 * ============================================================ */

int main(void)
{
    /* Seed the PRNG once at program start */
    srand((unsigned int)time(NULL));

    printf("\n");
    printf("  Welcome to the Automated Robotic Warehouse DES\n");
    printf("  B.Sc. Computer Science — Discrete-Event Simulation\n");

    int running = 1;
    while (running) {
        show_menu();

        int choice = 0;
        if (scanf("%d", &choice) != 1) {
            fprintf(stderr, "  Invalid input. Please enter 1-4.\n");
            flush_input_buffer();
            continue;
        }
        flush_input_buffer();

        SimContext *ctx = NULL;

        switch (choice) {
            case 1:
                ctx = get_simulation_params_interactive();
                run_simulation(ctx);
                cleanup_simulation(ctx);
                break;

            case 2:
                printf("\n  Loading '%s'...\n", DEFAULT_MAP_FILE);
                ctx = load_warehouse_config(DEFAULT_MAP_FILE);
                if (ctx) {
                    print_warehouse(ctx->warehouse);
                    printf("  Loaded: %d robots | time limit %.2f | "
                           "avg interval %.2f\n\n",
                           ctx->num_robots, ctx->time_limit,
                           ctx->avg_order_interval);
                    run_simulation(ctx);
                    cleanup_simulation(ctx);
                } else {
                    printf("  Could not load '%s'.\n", DEFAULT_MAP_FILE);
                    printf("  Use option 3 to generate a default file first.\n");
                }
                break;

            case 3:
                printf("\n");
                generate_default_config_file(DEFAULT_MAP_FILE);
                break;

            case 4:
                printf("\n  Exiting simulation. Goodbye!\n\n");
                running = 0;
                break;

            default:
                fprintf(stderr,
                    "  Unknown choice '%d'. Please enter 1-4.\n", choice);
                break;
        }
    }

    return EXIT_SUCCESS;
}
