#ifndef TYPES_H
#define TYPES_H

/*
 * types.h
 * -------
 * Central type definitions for the Automated Robotic Warehouse DES.
 * All enumerations, preprocessor constants, and data structures live here
 * so that every module shares a single, consistent set of types and avoids
 * circular include dependencies.
 *
 * Dependency: only standard C headers (<stdio.h> for FILE*).
 */

#include <stdio.h>

/* ============================================================
 * Preprocessor Constants
 * ============================================================ */

#define MAX_ROBOTS           20       /* Maximum robots the simulation supports    */
#define MAX_ORDERS           1000     /* Soft cap on total orders (for logging)     */
#define MAX_ID_LEN           32       /* Buffer size for entity ID strings          */
#define MAX_STATUS_LEN       128      /* Buffer size for status description strings */

#define BATTERY_LOW_THRESH   20       /* Battery % at or below which robot charges  */
#define BATTERY_PER_STEP     5        /* Battery % drained per BFS grid step        */
#define CHARGE_DURATION      10.0     /* Simulated time units to fully charge       */
#define TIME_PER_STEP        1.0      /* Simulated time units per grid movement step*/

#define DEFAULT_MAP_FILE     "warehouse_config.txt"
#define DEFAULT_LOG_FILE     "simulation_log.txt"
#define DEFAULT_REPORT_FILE  "simulation_report.txt"

/* ============================================================
 * Enumerations
 * ============================================================ */

/* Spatial tile types for the warehouse grid */
typedef enum {
    TILE_FLOOR    = 0,
    TILE_WALL     = 1,
    TILE_SHELF    = 2,
    TILE_PACKING  = 3,
    TILE_CHARGING = 4
} TileType;

/* Robot operational state machine */
typedef enum {
    ROBOT_IDLE              = 0,
    ROBOT_MOVING_TO_SHELF   = 1,
    ROBOT_MOVING_TO_PACKING = 2,
    ROBOT_CHARGING          = 3
} RobotState;

/* Order lifecycle state machine */
typedef enum {
    ORDER_PENDING     = 0,
    ORDER_IN_PROGRESS = 1,
    ORDER_COMPLETED   = 2
} OrderState;

/* Discrete event types managed by the Future Event List */
typedef enum {
    EVENT_ORDER_ARRIVAL           = 0,
    EVENT_ROBOT_REACHED_SHELF     = 1,
    EVENT_ROBOT_REACHED_PACKING   = 2,
    EVENT_ROBOT_CHARGING_COMPLETE = 3
} EventType;

/* ============================================================
 * Forward Typedefs
 * Resolve cross-struct pointer dependencies without full definitions.
 * ============================================================ */

typedef struct Order     Order;
typedef struct EventNode EventNode;

/* ============================================================
 * Core Data Structures
 * Defined in dependency order so later structs can reference earlier ones.
 * ============================================================ */

/* 2D integer grid coordinate */
typedef struct {
    int x; /* column (horizontal axis) */
    int y; /* row    (vertical   axis) */
} Position;

/* Dynamic 2D warehouse tile grid: grid[row][col] */
typedef struct {
    int        width;
    int        height;
    TileType **grid; /* heap-allocated; freed by destroy_warehouse() */
} Warehouse;

/* Autonomous mobile robot entity */
typedef struct {
    char      id[MAX_ID_LEN];           /* e.g. "R001"                         */
    Position  pos;                      /* Current grid position               */
    int       battery;                  /* Remaining charge: 0-100 %           */
    RobotState state;
    char      status_desc[MAX_STATUS_LEN]; /* Human-readable current activity  */
    int       orders_completed;
    int       charging_cycles;
    double    total_busy_time;          /* Accumulated active seconds          */
    double    busy_start_time;          /* Clock when current task started     */
    Order    *current_order;            /* NULL when IDLE or CHARGING          */
} Robot;

/* Customer order — also serves as node in the pending queue linked list */
struct Order {
    char       id[MAX_ID_LEN];   /* e.g. "O0001"                              */
    Position   shelf_pos;        /* Target shelf to retrieve from             */
    OrderState state;
    double     arrival_time;
    double     completion_time;
    struct Order *next;          /* Pending-queue intrusive linkage            */
};

/* FIFO queue for orders awaiting an idle robot */
typedef struct {
    Order *head;
    Order *tail;
    int    count;
} OrderQueue;

/* Immutable descriptor for a scheduled event in the FEL */
typedef struct {
    EventType type;
    double    time;       /* Simulation clock time when event fires */
    int       robot_idx; /* Index into SimContext.robots; -1 if unused */
    int       order_id;  /* Numeric order ID for logging;  -1 if unused */
} EventData;

/* Singly-linked FEL node (sorted ascending by EventData.time) */
struct EventNode {
    EventData         data;
    struct EventNode *next;
};

/* Future Event List — always ordered by ascending simulation time */
typedef struct {
    EventNode *head;
    int        count;
} FEL;

/* Aggregated statistics collected during the simulation run */
typedef struct {
    int    total_orders_processed;
    int    total_orders_arrived;
    double total_waiting_time;
    int    max_queue_length;
    int    current_queue_length;
} SimStats;

/* Master simulation context — passed by pointer to every subsystem */
typedef struct {
    Warehouse  *warehouse;
    Robot     **robots;             /* Heap array of Robot*; length = num_robots */
    int         num_robots;
    OrderQueue *pending_orders;
    FEL        *fel;
    SimStats   *stats;
    double      current_time;
    double      time_limit;
    double      avg_order_interval;
    int         next_order_id;      /* Monotonically incremented per order       */
    int         next_event_id;      /* Incremented per dispatch, used in logging */
    FILE       *log_file;           /* Opened in run_simulation, closed in cleanup */
} SimContext;

#endif /* TYPES_H */
