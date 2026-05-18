#define _CRT_SECURE_NO_WARNINGS

/*
 * stats.c
 * -------
 * Implements simulation statistics collection and the formatted final report.
 *
 * The internal write_report helper writes to an arbitrary FILE* so that
 * print_final_report can reuse it for both stdout and an optional log file
 * without duplicating the format strings.
 */

#include "stats.h"
#include "robot.h"
#include <stdio.h>
#include <stdlib.h>

SimStats *create_stats(void)
{
    SimStats *s = (SimStats *)calloc(1, sizeof(SimStats));
    if (!s) {
        fprintf(stderr, "ERROR: Failed to allocate SimStats.\n");
        exit(EXIT_FAILURE);
    }
    return s; /* calloc zeroes all fields */
}

void destroy_stats(SimStats *stats)
{
    if (!stats) return;
    free(stats);
}

void update_order_completed(SimStats *stats, double waiting_time)
{
    stats->total_orders_processed++;
    stats->total_waiting_time += waiting_time;
}

void update_queue_length(SimStats *stats, int current_len)
{
    stats->current_queue_length = current_len;
    if (current_len > stats->max_queue_length) {
        stats->max_queue_length = current_len;
    }
}

/* ============================================================
 * Internal Helper
 * ============================================================ */

/*
 * write_report
 * ------------
 * Writes the full formatted report to an arbitrary FILE* (stdout or a file).
 * Factored out so print_final_report can call it twice without duplicating
 * all the format strings.
 *
 * Parameters:
 *   out        - destination FILE*
 *   stats      - valid SimStats*
 *   robots     - array of Robot* of length num_robots
 *   num_robots - number of robots
 *   sim_time   - elapsed simulation time
 */
static void write_report(FILE *out, const SimStats *stats, Robot **robots,
                         int num_robots, double sim_time)
{
    int pending = stats->total_orders_arrived - stats->total_orders_processed;
    if (pending < 0) pending = 0;

    double avg_wait = (stats->total_orders_processed > 0)
        ? stats->total_waiting_time / (double)stats->total_orders_processed
        : 0.0;

    int i;

    fprintf(out, "\n");
    fprintf(out, "====================================================\n");
    fprintf(out, "           SIMULATION SUMMARY REPORT\n");
    fprintf(out, "====================================================\n");
    fprintf(out, "  Simulation Duration     : %8.2f time units\n", sim_time);
    fprintf(out, "  Total Orders Arrived    : %8d\n",
            stats->total_orders_arrived);
    fprintf(out, "  Total Orders Completed  : %8d\n",
            stats->total_orders_processed);
    fprintf(out, "  Orders Pending at End   : %8d\n", pending);
    fprintf(out, "  Average Waiting Time    : %8.2f time units\n", avg_wait);
    fprintf(out, "  Maximum Queue Length    : %8d\n",
            stats->max_queue_length);
    fprintf(out, "----------------------------------------------------\n");
    fprintf(out, "  ROBOT PERFORMANCE BREAKDOWN\n");
    fprintf(out, "----------------------------------------------------\n");

    for (i = 0; i < num_robots; i++) {
        Robot *r = robots[i];
        double util = (sim_time > 0.0)
            ? (r->total_busy_time / sim_time) * 100.0
            : 0.0;
        if (util > 100.0) util = 100.0;

        fprintf(out,
            "  Robot %-6s | Completed: %3d | "
            "Charging Cycles: %2d | Utilization: %5.1f%%\n",
            r->id, r->orders_completed, r->charging_cycles, util);
    }

    fprintf(out, "====================================================\n\n");
}

void print_final_report(const SimStats *stats, Robot **robots, int num_robots,
                        double sim_time, FILE *fp)
{
    write_report(stdout, stats, robots, num_robots, sim_time);
    if (fp) write_report(fp, stats, robots, num_robots, sim_time);
}

void save_report_to_file(const SimStats *stats, Robot **robots, int num_robots,
                         double sim_time, const char *report_path)
{
    FILE *fp = fopen(report_path, "w");
    if (!fp) {
        fprintf(stderr,
            "WARNING: Could not open report file '%s' for writing.\n",
            report_path);
        return;
    }
    write_report(fp, stats, robots, num_robots, sim_time);
    fclose(fp);
    printf("  [Report saved to '%s']\n", report_path);
}
