/*
 * Standalone PC/ESP unit test for ICP
 */

#include <stdio.h>
#include "icp.h"

int main(int argc, char *argv[]) {
    // Generate synthetic cloud 1 (L-shaped wall)
    const int N = 32;
    icp_points_t *cloud1 = points_alloc(N);
    icp_points_t *cloud2 = points_alloc(N);

    // Cloud 1: line from (0,0) to (1,0) and (1,0) to (1,1)
    for (int i = 0; i < 16; i++) {
        cloud1->items[i].x = (float) i / 15.0f;
        cloud1->items[i].y = 0.0f;
    }
    for (int i = 0; i < 16; i++) {
        cloud1->items[16 + i].x = 1.0f;
        cloud1->items[16 + i].y = (float) i / 15.0f;
    }

    // Cloud 2: Cloud 1 rotated by 5 deg and translated by (0.1, 0.05)
    float true_yaw = 5.0f * (float) M_PI / 180.0f;
    float true_tx = 0.10f;
    float true_ty = 0.05f;
    float c = cosf(true_yaw);
    float s = sinf(true_yaw);

    for (int i = 0; i < N; i++) {
        float x = cloud1->items[i].x;
        float y = cloud1->items[i].y;
        cloud2->items[i].x = x * c - y * s + true_tx;
        cloud2->items[i].y = x * s + y * c + true_ty;
    }

    icp_point_t trans = {0};
    float rot = 0;
    icp(cloud1, cloud2, 30, 0.5f, &trans, &rot);

    printf("ICP Alignment Result:\n");
    printf("  Translation (x, y): (%.4f, %.4f) [Ground Truth: (%.4f, %.4f)]\n",
           trans.x, trans.y, true_tx, true_ty);
    printf("  Rotation (deg):     %.3f [Ground Truth: %.3f]\n",
           rot * 180.0f / (float) M_PI, true_yaw * 180.0f / (float) M_PI);

    free(cloud1);
    free(cloud2);
    return 0;
}
