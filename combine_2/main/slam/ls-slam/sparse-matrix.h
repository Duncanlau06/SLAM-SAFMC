/*
 * Copyright (C) 2023 ETH Zurich
 * Adapted for ESP32-S3 Autonomous Drone Swarm SLAM
 */

#ifndef SPARSEMAT_H
#define SPARSEMAT_H

#include <stdint.h>
#include "config_size.h"
#include "run_on_pc.h"

typedef struct {
    float data[MAX_SIZE_SP_MAT];
    int16_t col[MAX_SIZE_SP_MAT];
    int16_t rowCnt[MAX_SIZE + 1];
    int32_t idx0;
    int16_t idx1;
    int32_t highestOrder;
} sparseMatrix;

void insert_element(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j, float element);
void add_to_element(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j, float value);
int16_t already_added(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j);
void sparse_from_dense(int16_t N, float mat[N][N], sparseMatrix *spMat);
float get_element(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j);
float get_element_sym(sparseMatrix *spMat, int16_t N, int16_t i, int16_t j);
void transpose_lower_triangular(sparseMatrix *spMat, int16_t N);

#endif // SPARSEMAT_H
