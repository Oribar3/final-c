#define _CRT_SECURE_NO_WARNINGS

/*
 * pathfinding.c
 * -------------
 * Implements BFS-based pathfinding on the warehouse 2D grid.
 *
 * Design decisions:
 *   - BFS guarantees the shortest path on an unweighted grid.
 *   - All auxiliary arrays (visited, queue) are heap-allocated to avoid
 *     stack overflow on large warehouse configurations.
 *   - The queue is a flat Position array of size width*height, which is
 *     the maximum number of tiles ever enqueued before re-visiting begins.
 *   - A flat visited/distance array indexed as [y * width + x] avoids
 *     a secondary 2D allocation and improves cache locality.
 */

#include "pathfinding.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Four cardinal movement directions: right, left, down, up */
static const int DX[4] = { 1, -1,  0, 0 };
static const int DY[4] = { 0,  0,  1,-1 };

/* ============================================================
 * bfs_distance
 * ============================================================ */

int bfs_distance(const Warehouse *wh, Position start, Position end)
{
    /* Validate inputs */
    if (!is_valid_pos(wh, start.x, start.y) ||
        !is_valid_pos(wh, end.x,   end.y))  return -1;
    if (!is_walkable(wh, end.x, end.y))      return -1;
    if (start.x == end.x && start.y == end.y) return 0;

    int total = wh->width * wh->height;

    /* Heap-allocate distance array: -1 = unvisited */
    int *dist = (int *)malloc((size_t)total * sizeof(int));
    if (!dist) {
        fprintf(stderr, "ERROR: bfs_distance: dist allocation failed.\n");
        return -1;
    }
    int i;
    for (i = 0; i < total; i++) dist[i] = -1;

    /* Heap-allocate BFS queue */
    Position *queue = (Position *)malloc((size_t)total * sizeof(Position));
    if (!queue) {
        fprintf(stderr, "ERROR: bfs_distance: queue allocation failed.\n");
        free(dist);
        return -1;
    }

    int head = 0, tail = 0;
    dist[start.y * wh->width + start.x] = 0;
    queue[tail].x = start.x;
    queue[tail].y = start.y;
    tail++;

    while (head < tail) {
        Position cur = queue[head++];
        int cur_dist = dist[cur.y * wh->width + cur.x];

        int d;
        for (d = 0; d < 4; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];

            if (!is_walkable(wh, nx, ny))             continue;
            if (dist[ny * wh->width + nx] != -1)      continue;

            dist[ny * wh->width + nx] = cur_dist + 1;

            if (nx == end.x && ny == end.y) {
                int result = dist[ny * wh->width + nx];
                free(dist);
                free(queue);
                return result;
            }

            queue[tail].x = nx;
            queue[tail].y = ny;
            tail++;
        }
    }

    free(dist);
    free(queue);
    return -1; /* Destination unreachable */
}

/* ============================================================
 * find_nearest_tile_bfs
 * ============================================================ */

Position find_nearest_tile_bfs(const Warehouse *wh, Position start,
                                TileType target_type)
{
    Position not_found;
    not_found.x = -1;
    not_found.y = -1;

    if (!is_valid_pos(wh, start.x, start.y)) return not_found;

    /* If the start tile itself is the target, return immediately */
    if (get_tile(wh, start.x, start.y) == target_type) return start;

    int total = wh->width * wh->height;

    /* Heap-allocate visited flags */
    int *visited = (int *)calloc((size_t)total, sizeof(int));
    if (!visited) {
        fprintf(stderr, "ERROR: find_nearest_tile_bfs: visited allocation failed.\n");
        return not_found;
    }

    /* Heap-allocate BFS queue */
    Position *queue = (Position *)malloc((size_t)total * sizeof(Position));
    if (!queue) {
        fprintf(stderr, "ERROR: find_nearest_tile_bfs: queue allocation failed.\n");
        free(visited);
        return not_found;
    }

    int head = 0, tail = 0;
    visited[start.y * wh->width + start.x] = 1;
    queue[tail].x = start.x;
    queue[tail].y = start.y;
    tail++;

    while (head < tail) {
        Position cur = queue[head++];

        int d;
        for (d = 0; d < 4; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];

            if (!is_walkable(wh, nx, ny))          continue;
            if (visited[ny * wh->width + nx])       continue;

            visited[ny * wh->width + nx] = 1;

            if (get_tile(wh, nx, ny) == target_type) {
                Position result;
                result.x = nx;
                result.y = ny;
                free(visited);
                free(queue);
                return result;
            }

            queue[tail].x = nx;
            queue[tail].y = ny;
            tail++;
        }
    }

    free(visited);
    free(queue);
    return not_found;
}
