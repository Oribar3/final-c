#ifndef FILEIO_H
#define FILEIO_H

/*
 * fileio.h
 * --------
 * Declares the file I/O API:
 *   - Loading a warehouse configuration from a structured text file.
 *   - Writing a variadic event log entry to both stdout and a log file.
 *   - Opening and closing the simulation event log.
 *   - Generating a ready-to-use default configuration file.
 *
 * Configuration file format:
 *   WIDTH  <int>
 *   HEIGHT <int>
 *   NUM_ROBOTS <int>
 *   TIME_LIMIT <double>
 *   AVG_ORDER_INTERVAL <double>
 *   MAP
 *   <height rows, each exactly width characters>
 *
 *   Tile characters: W=wall  F=floor  S=shelf  P=packing  C=charging
 *
 * Depends on: types.h
 */

#include "types.h"

/*
 * load_warehouse_config
 * ---------------------
 * Parses the configuration file at filename and assembles a fully initialised
 * SimContext. Robot starting positions are the first NUM_ROBOTS TILE_FLOOR
 * tiles encountered in row-major (top-to-bottom, left-to-right) order.
 *
 * Validation performed:
 *   - File must be openable.
 *   - WIDTH and HEIGHT must be set before MAP.
 *   - The grid must contain at least one SHELF, PACKING, and CHARGING tile
 *     (simulation cannot run without them).
 *   - Unknown tile characters are treated as FLOOR with a stderr warning.
 *
 * Parameters:
 *   filename - path to the configuration .txt file
 *
 * Returns:
 *   Fully initialised SimContext* on success.
 *   NULL if the file cannot be opened (caller should inform user).
 *   Calls exit(EXIT_FAILURE) on structural/parse errors.
 */
SimContext *load_warehouse_config(const char *filename);

/*
 * save_event_log_entry
 * --------------------
 * Writes a printf-style formatted message to stdout and, if fp is non-NULL,
 * to the given log file. Uses two independent va_list instances so the
 * variadic arguments are consumed correctly by each vprintf call.
 *
 * Parameters:
 *   fp     - open log FILE*; may be NULL for stdout-only output
 *   format - printf-style format string
 *   ...    - variadic format arguments
 */
void save_event_log_entry(FILE *fp, const char *format, ...);

/*
 * open_log_file
 * -------------
 * Opens filename for writing (creates or truncates the file).
 *
 * Parameters:
 *   filename - path to the log file
 *
 * Returns:
 *   FILE* on success; NULL on failure (error reported to stderr).
 */
FILE *open_log_file(const char *filename);

/*
 * close_log_file
 * --------------
 * Flushes and closes the log file. No-op if fp is NULL.
 *
 * Parameters:
 *   fp - open FILE* or NULL
 */
void close_log_file(FILE *fp);

/*
 * generate_default_config_file
 * ----------------------------
 * Writes a ready-to-run warehouse_config.txt with a 10-column × 8-row
 * sample layout featuring shelves, packing stations, and charging stations.
 * Creates or overwrites the file; errors reported to stderr.
 *
 * Parameters:
 *   filename - destination path; uses DEFAULT_MAP_FILE if NULL.
 */
void generate_default_config_file(const char *filename);

#endif /* FILEIO_H */
