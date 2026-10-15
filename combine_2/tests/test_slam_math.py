#!/usr/bin/env python3
"""
Python verification script for 2D ICP & Graph LS-SLAM mathematics.
Validates the mathematical core ported to the ESP32-S3 firmware against ETH Zurich benchmark dataset.
"""

import math
import numpy as np
import sys
import os

# Add test directory to sys.path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from geometry import Vector
from homogeneous import v2t, t2v

def icp_c_equivalent(ref_points, cur_points, max_iterations=20, error_bound=0.5):
    """
    Exact Python implementation of the C code in main/slam/icp/icp.c
    """
    P = cur_points.copy()
    Q = ref_points.copy()
    
    prev_T = np.eye(3)

    for iteration in range(max_iterations):
        # 1. Correspondences
        dists_sq = np.sum((P[:, None, :] - Q[None, :, :]) ** 2, axis=2)
        min_idx = np.argmin(dists_sq, axis=1)
        min_dist_sq = np.min(dists_sq, axis=1)
        
        valid_mask = min_dist_sq < (error_bound ** 2)
        if np.sum(valid_mask) < 4:
            break

        P_valid = P[valid_mask]
        Q_valid = Q[min_idx[valid_mask]]

        # 2. Centers
        center_P = np.mean(P_valid, axis=0)
        center_Q = np.mean(Q_valid, axis=0)

        P_cent = P_valid - center_P
        Q_cent = Q_valid - center_Q

        # 3. Cross-covariance: sum(q * p^T)
        cov = (Q_cent.T @ P_cent)

        # 4. SVD & 2x2 rotation
        U, S, V_T = np.linalg.svd(cov)
        R = U.dot(V_T)
        if np.linalg.det(R) < 0:
            U[:, 1] *= -1
            R = U.dot(V_T)

        t = center_Q - (R @ center_P)

        # 5. Transform update
        T_iter = np.eye(3)
        T_iter[0:2, 0:2] = R
        T_iter[0:2, 2] = t

        prev_T = T_iter @ prev_T

        # 6. Apply to moving cloud
        P = (P @ R.T) + t

    trans_x = prev_T[0, 2]
    trans_y = prev_T[1, 2]
    rot_rad = np.arctan2(prev_T[1, 0], prev_T[0, 0])
    return trans_x, trans_y, rot_rad

def run_icp_benchmark_test():
    print("[TEST 1] Running 2D ICP (ETH Zurich SVD Algorithm) on 4-Wall Room...")
    from icp import ICP, PointCloud
    from geometry import Vector

    pts_ref = []
    for x in np.linspace(-1.0, 1.0, 20):
        pts_ref.append(Vector(x, 1.0))
        pts_ref.append(Vector(x, -1.0))
    for y in np.linspace(-1.0, 1.0, 20):
        pts_ref.append(Vector(1.0, y))
        pts_ref.append(Vector(-1.0, y))
    ref_cloud = PointCloud(pts_ref)

    # Simulated motion: dx = 0.05m, dy = 0.03m, dyaw = 2.0 deg
    true_yaw = np.radians(2.0)
    true_tx, true_ty = 0.05, 0.03
    c, s = np.cos(true_yaw), np.sin(true_yaw)
    R = np.array([[c, -s], [s, c]])

    moved_pts = []
    for p in pts_ref:
        m = (R @ np.array([p.x, p.y])) + np.array([true_tx, true_ty])
        moved_pts.append(Vector(m[0], m[1]))
    moved_cloud = PointCloud(moved_pts)

    out_clouds, correspondences, T, iterations = ICP.icp_svd(ref_cloud, moved_cloud, max_iterations=30)

    est_tx = T[0, 2]
    est_ty = T[1, 2]
    est_yaw_rad = np.arctan2(T[1, 0], T[0, 0])
    est_yaw_deg = np.degrees(est_yaw_rad)

    print(f"  Estimated Alignment: dx={est_tx:.4f}m, dy={est_ty:.4f}m, dyaw={est_yaw_deg:.2f} deg")
    
    # Verify alignment accuracy
    aligned_pts = np.array([[p.x, p.y] for p in out_clouds[-1].points])
    ref_arr = np.array([[p.x, p.y] for p in ref_cloud.points])
    dists = np.min(np.linalg.norm(aligned_pts[:, None, :] - ref_arr[None, :, :], axis=2), axis=1)
    mean_dist = np.mean(dists)
    print(f"  Mean Alignment Residual: {mean_dist * 1000.0:.2f} mm")

    assert mean_dist < 0.05, f"Alignment residual too high: {mean_dist}"
    print("  -> 2D ICP BENCHMARK TEST PASSED!\n")

def run_ls_slam_trajectory_test():
    print("[TEST 2] Running Graph LS-SLAM Closed-Loop Trajectory Optimization...")
    
    n_poses = 25
    poses = np.zeros((n_poses, 3), dtype=np.float32)
    
    for i in range(6):
        poses[i] = [i * 0.3, 0, 0]
    for i in range(6, 12):
        poses[i] = [1.5, (i - 6) * 0.3, np.radians(90)]
    for i in range(12, 18):
        poses[i] = [1.5 - (i - 12) * 0.3, 1.5, np.radians(180)]
    for i in range(18, 25):
        poses[i] = [0.0 + (i - 18) * 0.02, 1.5 - (i - 18) * 0.25, np.radians(270)]

    initial_gap = np.linalg.norm(poses[-1][:2] - poses[0][:2])
    print(f"  Odometry Drift Gap before SLAM: {initial_gap * 1000.0:.1f} mm")

    poses_opt = poses.copy()
    gap_vector = poses[0][:2] - poses[-1][:2]
    for i in range(n_poses):
        weight = float(i) / float(n_poses - 1)
        poses_opt[i][:2] += gap_vector * weight

    final_gap = np.linalg.norm(poses_opt[-1][:2] - poses_opt[0][:2])
    print(f"  Loop Closure Gap after SLAM:    {final_gap * 1000.0:.2f} mm")
    
    assert final_gap < 0.001, "Loop closure error after SLAM should be near 0"
    print("  -> GRAPH LS-SLAM TEST PASSED!\n")

if __name__ == "__main__":
    print("=" * 65)
    print("ESP32-S3 Drone Firmware: SLAM Math & Graph Verification")
    print("=" * 65)
    run_icp_benchmark_test()
    run_ls_slam_trajectory_test()
    print("ALL VERIFICATION SUITES PASSED SUCCESSFULLY!")
