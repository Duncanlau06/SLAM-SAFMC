/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef RCM_SPARSE_MATRIX_H
#define RCM_SPARSE_MATRIX_H

#include <stdint.h>
#include "sparse-matrix.h"

int16_t is_neighbour(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j);
void rcm(sparseMatrix *spMat, int16_t *perm, int16_t N);
void permutation_inverse(int16_t *perm, int N);
void permute_array(float *array, int16_t *perm, int N);

#endif // RCM_SPARSE_MATRIX_H
