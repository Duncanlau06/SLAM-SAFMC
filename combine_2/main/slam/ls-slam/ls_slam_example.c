/*
 * Standalone test for Graph-Based LS-SLAM
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "graph-based-slam.h"

int main(int argc, char *argv[]) {
    int16_t nr_of_poses = 20;
    float x_cmd = 0, y_cmd = 0, yaw_cmd = 0;
    float edge_l = 0;
    float edge_w = 0;

    float x[3 * nr_of_poses];
    memset(x, 0, sizeof(x));

    for (int16_t i = 0; i < nr_of_poses; i++) {
        yaw_cmd = i * 0.01f;
        if (i < 5) {
            x_cmd = edge_l / 10.0f;
            y_cmd = 0;
            edge_l += 1;
        } else if (i < 10) {
            x_cmd = edge_l / 10.0f;
            y_cmd = edge_w / 10.0f;
            edge_w += 1;
        } else if (i < 15) {
            edge_l -= 1;
            x_cmd = edge_l / 10.0f;
            y_cmd = edge_w / 10.0f;
        } else {
            edge_w -= 1;
            x_cmd = edge_l / 10.0f;
            y_cmd = edge_w / 10.0f;
        }
        x[3 * i] = x_cmd;
        x[3 * i + 1] = y_cmd;
        x[3 * i + 2] = yaw_cmd;
    }

    constraint constraints[1];
    memset(constraints, 0, sizeof(constraints));
    constraints[0].fromIdx = nr_of_poses - 1;
    constraints[0].toIdx = 0;
    constraints[0].z[0] = 0.0f;
    constraints[0].z[1] = 0.0f;
    constraints[0].z[2] = 0.0f;

    uint8_t skip[nr_of_poses];
    memset(skip, 0, sizeof(skip));

    printf("Running LS-SLAM Optimization on %d poses with 1 loop closure...\n", nr_of_poses);
    ls_slam(x, skip, constraints, nr_of_poses, 1);

    printf("Optimized Poses (x, y, yaw):\n");
    for (int16_t i = 0; i < nr_of_poses; i++) {
        printf("  Pose %2d: x=%.3f, y=%.3f, yaw=%.3f\n",
               i, x[3 * i], x[3 * i + 1], x[3 * i + 2]);
    }
    printf("LS-SLAM successfully optimized graph.\n");
    return 0;
}
