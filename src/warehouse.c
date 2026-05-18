#define _CRT_SECURE_NO_WARNINGS

/*
 * warehouse.c
 * -----------
 * Implements the dynamic 2D warehouse tile grid.
 * The grid is a TileType** allocated as height independent rows so that
 * each row can be freed individually, satisfying the project requirement
 * for a proper dynamic matrix with explicit memory management.
 */

#include "warehouse.h"
#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 * Internal Helper
 * ============================================================ */

/*
 * tile_to_char
 * ------------
 * Maps a TileType enum value to its single display character.
 * Uses a switch so adding new tile types never breaks silently.
 *
 * Parameters:
 *   t - TileType value
 *
 * Returns: display character (W/F/S/P/C or '?' for unknown).
 */
static char tile_to_char(TileType t)
{
    switch (t) {
        case TILE_FLOOR:    return 'F';
        case TILE_WALL:     return 'W';
        case TILE_SHELF:    return 'S';
        case TILE_PACKING:  return 'P';
        case TILE_CHARGING: return 'C';
        default:            return '?';
    }
}

/* ============================================================
 * Public API
 * ============================================================ */

Warehouse *create_warehouse(int width, int height)
{
    Warehouse *wh = (Warehouse *)malloc(sizeof(Warehouse));
    if (!wh) {
        fprintf(stderr, "ERROR: Failed to allocate Warehouse struct.\n");
        exit(EXIT_FAILURE);
    }
    wh->width  = width;
    wh->height = height;

    /* Allocate the row-pointer array */
    wh->grid = (TileType **)calloc((size_t)height, sizeof(TileType *));
    if (!wh->grid) {
        fprintf(stderr, "ERROR: Failed to allocate warehouse grid row array.\n");
        free(wh);
        exit(EXIT_FAILURE);
    }

    /* Allocate and zero-fill each row individually */
    int y;
    for (y = 0; y < height; y++) {
        wh->grid[y] = (TileType *)calloc((size_t)width, sizeof(TileType));
        if (!wh->grid[y]) {
            fprintf(stderr, "ERROR: Failed to allocate warehouse grid row %d.\n", y);
            /* Free all rows already allocated before exiting */
            int i;
            for (i = 0; i < y; i++) free(wh->grid[i]);
            free(wh->grid);
            free(wh);
            exit(EXIT_FAILURE);
        }
        /* calloc zeroes memory; TILE_FLOOR == 0, so all tiles start as floor */
    }

    return wh;
}

void destroy_warehouse(Warehouse *wh)
{
    if (!wh) return;

    /* Free each row first, then the pointer array, then the struct */
    int y;
    for (y = 0; y < wh->height; y++) {
        free(wh->grid[y]);
    }
    free(wh->grid);
    free(wh);
}

TileType get_tile(const Warehouse *wh, int x, int y)
{
    return wh->grid[y][x];
}

void set_tile(Warehouse *wh, int x, int y, TileType type)
{
    wh->grid[y][x] = type;
}

int is_valid_pos(const Warehouse *wh, int x, int y)
{
    return (x >= 0 && x < wh->width && y >= 0 && y < wh->height);
}

int is_walkable(const Warehouse *wh, int x, int y)
{
    if (!is_valid_pos(wh, x, y)) return 0;
    return wh->grid[y][x] != TILE_WALL;
}

void print_warehouse(const Warehouse *wh)
{
    int x, y;
    printf("\n  +-- Warehouse Map (%d x %d) --+\n", wh->width, wh->height);
    printf("     ");
    for (x = 0; x < wh->width; x++) printf("%d", x % 10);
    printf("\n");

    for (y = 0; y < wh->height; y++) {
        printf("  %2d ", y);
        for (x = 0; x < wh->width; x++) {
            printf("%c", tile_to_char(wh->grid[y][x]));
        }
        printf("\n");
    }
    printf("  Legend: W=Wall  F=Floor  S=Shelf  P=Packing  C=Charging\n\n");
}
