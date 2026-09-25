/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef GRAPHBASEDSLAM_H
#define GRAPHBASEDSLAM_H

#include <stdint.h>
#include "sparse-matrix.h"
#include "run_on_pc.h"

typedef struct {
    int16_t fromIdx;
    int16_t toIdx;
    float z[3];
} constraint;

typedef struct {
    float z[3];
} measurement;

void linearize_pose_pose_constraint(float *x1, float *x2, float *z, float A[3][3], float B[3][3], float *e);

void build_H_and_b(float *x,
                   sparseMatrix *H,
                   float *b,
                   measurement *measurements,
                   const uint8_t *skip,
                   constraint *constraints,
                   int16_t nr_of_poses,
                   int16_t nr_of_constr);

void solve_for_x(sparseMatrix *H, float *b, int16_t *perm, int16_t size);

void measurement_from_poses(float *x_from, float *x_to, float *z);

void measurements_from_poses(float *x, measurement *measurements, int nr_of_poses);

void forward_substitution(sparseMatrix *L, float *b, int N);

void backward_substitution(sparseMatrix *L, float *b, int N);

void ls_slam(float *x, uint8_t *skip, constraint *constraints, int16_t nr_of_poses, int16_t nr_of_constr);

float test_linear_system(sparseMatrix *H, float *x, float *b, int N);

#endif // GRAPHBASEDSLAM_H
