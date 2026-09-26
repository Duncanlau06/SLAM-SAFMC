/**
 * @file occ_grid.h
 * @brief Onboard occupancy-grid map, built from the optimised SLAM graph.
 *
 * After landing and LS-SLAM, every scan point is raycast from its (optimised)
 * pose: cells the beam passes through count as misses, the end cell as a hit.
 * Cells hit from several scans and rarely passed through become OCCUPIED; one-off
 * reflections and isolated specks are dropped. The grid is printed over the
 * console as a run-length-encoded block that plot_slam_map.py displays as-is.
 */

#ifndef OCC_GRID_H
#define OCC_GRID_H

#define OCC_RES_M            0.05f  /* cell size; coarsened if the area is too large */
#define OCC_MARGIN_M         0.20f  /* padding around the mapped area */
#define OCC_MAX_DIM          400    /* max cells per side (400x400 = 480 KB PSRAM) */
#define OCC_MIN_HITS         2      /* distinct scans that must hit a cell */
#define OCC_MIN_RATIO_PCT    35     /* hits / (hits + misses) to count as occupied */

/* Builds the grid from the swarm graph and prints it between
 * "=== SLAM_GRID_START ===" and "=== SLAM_GRID_END ===". Memory is allocated
 * in PSRAM and freed before returning. */
void occ_grid_build_and_export(void);

#endif // OCC_GRID_H
