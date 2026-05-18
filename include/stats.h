#ifndef STATS_H
#define STATS_H

/*
 * stats.h
 * -------
 * Declares the API for collecting simulation statistics and generating
 * the final analytical summary report printed to stdout and saved to file.
 *
 * Robot utilization is computed here from Robot.total_busy_time fields;
 * the stats module reads those fields but never writes them.
 *
 * Depends on: types.h
 */

#include "types.h"

/*
 * create_stats
 * ------------
 * Heap-allocates a SimStats struct initialised to all zeros via calloc.
 *
 * Returns: SimStats*; exits on allocation failure.
 */
SimStats *create_stats(void);

/*
 * destroy_stats
 * -------------
 * Frees the SimStats struct.
 *
 * Parameters:
 *   stats - may be NULL (no-op).
 */
void destroy_stats(SimStats *stats);

/*
 * update_order_completed
 * ----------------------
 * Records the completion of one order: increments the processed counter
 * and accumulates waiting_time for the running average.
 *
 * Parameters:
 *   stats        - valid SimStats*
 *   waiting_time - elapsed simulation time from order arrival to completion
 */
void update_order_completed(SimStats *stats, double waiting_time);

/*
 * update_queue_length
 * -------------------
 * Records the current pending-queue length and updates the running maximum.
 * Call this after every enqueue or dequeue operation.
 *
 * Parameters:
 *   stats       - valid SimStats*
 *   current_len - new queue length
 */
void update_queue_length(SimStats *stats, int current_len);

/*
 * print_final_report
 * ------------------
 * Renders the full simulation summary dashboard to stdout.
 * If fp is non-NULL the identical output is also written to that file.
 *
 * Parameters:
 *   stats      - valid SimStats*
 *   robots     - array of Robot* of length num_robots
 *   num_robots - number of robots
 *   sim_time   - simulation clock value at termination
 *   fp         - secondary output file (may be NULL)
 */
void print_final_report(const SimStats *stats, Robot **robots, int num_robots,
                        double sim_time, FILE *fp);

/*
 * save_report_to_file
 * -------------------
 * Opens report_path for writing, writes the full report, then closes it.
 * Errors are reported to stderr without crashing.
 *
 * Parameters:
 *   stats, robots, num_robots, sim_time - same semantics as print_final_report
 *   report_path - file path string (must not be NULL)
 */
void save_report_to_file(const SimStats *stats, Robot **robots, int num_robots,
                         double sim_time, const char *report_path);

#endif /* STATS_H */
