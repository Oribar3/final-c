#define _CRT_SECURE_NO_WARNINGS

/*
 * fileio.c
 * --------
 * Implements configuration file loading, the variadic event logger, and
 * the default config file generator.
 *
 * load_warehouse_config reads the file line by line, parsing key=value
 * pairs until the MAP keyword, then reading height rows of tile characters.
 * It assembles a complete SimContext so main.c does not need to know the
 * internals of any individual module.
 *
 * save_event_log_entry uses two separate va_start/va_end pairs because a
 * va_list object must not be used after va_end; restarting from the format
 * argument is the only portable way to print to two destinations.
 */

#include "../include/fileio.h"
#include "../include/warehouse.h"
#include "../include/robot.h"
#include "../include/order.h"
#include "../include/stats.h"
#include "../include/event.h"      /* create_fel */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#define MAX_LINE 512    /* Maximum character width of a config file line */

/* ============================================================
 * load_warehouse_config
 * ============================================================ */

SimContext *load_warehouse_config(const char *filename)
{
    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr,
            "ERROR: Cannot open configuration file '%s'.\n", filename);
        return NULL;
    }

    /* Parsed parameters */
    int    width       = 0;
    int    height      = 0;
    int    num_robots  = 0;
    double time_limit  = 0.0;
    double avg_interval= 0.0;

    char line[MAX_LINE];
    int  in_map    = 0;  /* 1 once the MAP token is seen */
    int  rows_read = 0;
    Warehouse *wh  = NULL;

    while (fgets(line, sizeof(line), fp)) {
        /* Strip trailing CR and LF characters */
        size_t len = strlen(line);
        while (len > 0 && (line[len-1] == '\n' || line[len-1] == '\r')) {
            line[--len] = '\0';
        }

        /* Skip blank lines and comment lines (starting with '#') */
        if (len == 0 || line[0] == '#') continue;

        if (!in_map) {
            /* Parse key-value configuration lines */
            if (strncmp(line, "WIDTH", 5) == 0) {
                sscanf(line + 5, " %d", &width);
            } else if (strncmp(line, "HEIGHT", 6) == 0) {
                sscanf(line + 6, " %d", &height);
            } else if (strncmp(line, "NUM_ROBOTS", 10) == 0) {
                sscanf(line + 10, " %d", &num_robots);
            } else if (strncmp(line, "TIME_LIMIT", 10) == 0) {
                sscanf(line + 10, " %lf", &time_limit);
            } else if (strncmp(line, "AVG_ORDER_INTERVAL", 18) == 0) {
                sscanf(line + 18, " %lf", &avg_interval);
            } else if (strcmp(line, "MAP") == 0) {
                if (width <= 0 || height <= 0) {
                    fprintf(stderr,
                        "ERROR: WIDTH and HEIGHT must be set before MAP.\n");
                    fclose(fp);
                    exit(EXIT_FAILURE);
                }
                wh = create_warehouse(width, height);
                in_map = 1;
            }
        } else {
            /* Parse one row of the map grid */
            if (rows_read >= height) continue; /* Ignore extra rows */

            int x;
            for (x = 0; x < width && line[x] != '\0'; x++) {
                TileType t;
                switch (line[x]) {
                    case 'W': t = TILE_WALL;     break;
                    case 'S': t = TILE_SHELF;    break;
                    case 'F': t = TILE_FLOOR;    break;
                    case 'P': t = TILE_PACKING;  break;
                    case 'C': t = TILE_CHARGING; break;
                    default:
                        fprintf(stderr,
                            "WARNING: Unknown tile char '%c' at row %d "
                            "col %d. Treating as FLOOR.\n",
                            line[x], rows_read, x);
                        t = TILE_FLOOR;
                        break;
                }
                set_tile(wh, x, rows_read, t);
            }
            rows_read++;
        }
    }
    fclose(fp);

    if (!wh) {
        fprintf(stderr,
            "ERROR: File '%s' did not contain a MAP section.\n", filename);
        return NULL;
    }

    /* Validate that the warehouse has all required tile types */
    int has_shelf = 0, has_packing = 0, has_charging = 0;
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            TileType t = get_tile(wh, x, y);
            if (t == TILE_SHELF)    has_shelf    = 1;
            if (t == TILE_PACKING)  has_packing  = 1;
            if (t == TILE_CHARGING) has_charging = 1;
        }
    }
    if (!has_shelf || !has_packing || !has_charging) {
        fprintf(stderr,
            "ERROR: Warehouse must contain at least one each of: "
            "S (shelf), P (packing), C (charging).\n");
        destroy_warehouse(wh);
        exit(EXIT_FAILURE);
    }

    /* Clamp robot count to valid range */
    if (num_robots < 1)        num_robots = 1;
    if (num_robots > MAX_ROBOTS) num_robots = MAX_ROBOTS;

    /* Allocate robot pointer array */
    Robot **robots = (Robot **)calloc((size_t)num_robots, sizeof(Robot *));
    if (!robots) {
        fprintf(stderr, "ERROR: Failed to allocate robots array.\n");
        destroy_warehouse(wh);
        exit(EXIT_FAILURE);
    }

    /* Place robots on the first num_robots FLOOR tiles in row-major order */
    int placed = 0;
    for (y = 0; y < height && placed < num_robots; y++) {
        for (x = 0; x < width && placed < num_robots; x++) {
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
            "ERROR: No FLOOR tiles found for robot placement. "
            "Check warehouse layout.\n");
        free(robots);
        destroy_warehouse(wh);
        exit(EXIT_FAILURE);
    }
    if (placed < num_robots) {
        fprintf(stderr,
            "WARNING: Only %d floor tiles found; using %d robots "
            "instead of %d.\n", placed, placed, num_robots);
        num_robots = placed;
    }

    /* Assemble the SimContext */
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
    ctx->num_robots         = num_robots;
    ctx->pending_orders     = create_order_queue();
    ctx->fel                = create_fel();
    ctx->stats              = create_stats();
    ctx->current_time       = 0.0;
    ctx->time_limit         = (time_limit  > 0.0) ? time_limit  : 100.0;
    ctx->avg_order_interval = (avg_interval> 0.0) ? avg_interval:   5.0;
    ctx->next_order_id      = 1;
    ctx->next_event_id      = 1;
    ctx->log_file           = NULL;

    return ctx;
}

/* ============================================================
 * Logging Utilities
 * ============================================================ */

void save_event_log_entry(FILE *fp, const char *format, ...)
{
    va_list args;

    /* First va_list: write to stdout */
    va_start(args, format);
    vprintf(format, args);
    va_end(args);

    /* Second va_list: write to file (cannot reuse the same va_list) */
    if (fp) {
        va_start(args, format);
        vfprintf(fp, format, args);
        va_end(args);
    }
}

FILE *open_log_file(const char *filename)
{
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr,
            "WARNING: Could not open log file '%s'. "
            "Logging to stdout only.\n", filename);
    }
    return fp;
}

void close_log_file(FILE *fp)
{
    if (!fp) return;
    fflush(fp);
    fclose(fp);
}

/* ============================================================
 * Default Configuration Generator
 * ============================================================ */

void generate_default_config_file(const char *filename)
{
    if (!filename) filename = DEFAULT_MAP_FILE;

    FILE *fp = fopen(filename, "w");
    if (!fp) {
        fprintf(stderr,
            "ERROR: Cannot create configuration file '%s'.\n", filename);
        return;
    }

    /*
     * Sample 10-wide x 8-tall warehouse:
     *   Row 0,7 : wall border
     *   Row 1,5 : shelves at cols 1,3,5,7 (interleaved with floor)
     *   Row 3   : packing stations at cols 1 and 8
     *   Row 6   : charging stations at cols 1 and 8
     *   All other interior tiles : floor
     */
    fprintf(fp,
        "# Automated Robotic Warehouse - Simulation Configuration\n"
        "# -------------------------------------------------------\n"
        "# Tile key: W=Wall  F=Floor  S=Shelf  P=Packing  C=Charging\n"
        "#\n"
        "WIDTH 10\n"
        "HEIGHT 8\n"
        "NUM_ROBOTS 3\n"
        "TIME_LIMIT 100.0\n"
        "AVG_ORDER_INTERVAL 5.0\n"
        "MAP\n"
        "WWWWWWWWWW\n"
        "WSFSFSFSFW\n"
        "WFFFFFFFFW\n"
        "WPFFFFFFPW\n"
        "WFFFFFFFFW\n"
        "WSFSFSFSFW\n"
        "WCFFFFFFCW\n"
        "WWWWWWWWWW\n"
    );

    fclose(fp);
    printf("  Default configuration written to '%s'.\n", filename);
    printf("  Warehouse: 10x8 | 3 robots | time limit 100 | "
           "avg arrival interval 5.0\n");
}
