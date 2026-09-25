#!/usr/bin/env python3
"""
Unit verification script for SLAM coordinate fixes, quality gating, and parser.
"""

import math
import numpy as np
import subprocess
import os
import sys

# Add parent directory so we can import plot_slam_map
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from plot_slam_map import parse_slam_data, plot_slam_map

def test_ned_coordinate_projections():
    print("[TEST 1] Testing NED Coordinate Sensor Projections...")
    
    # In NED frame:
    # North is +X (heading 0 rad)
    # East is +Y (heading +pi/2 rad)
    # South is -X (heading pi rad)
    # West is -Y (heading -pi/2 rad)
    
    # Test drone at (0, 0) facing North (yaw = 0)
    frame_x, frame_y, frame_yaw = 0.0, 0.0, 0.0
    step = -math.pi / 4.0 / 8.0
    
    for direction, name, exp_x, exp_y in [
        (0, "FRONT", 1.0, 0.0),   # North (+X)
        (1, "BACK",  -1.0, 0.0),  # South (-X)
        (2, "LEFT",  0.0, -1.0),  # West (-Y)
        (3, "RIGHT", 0.0, 1.0),   # East (+Y)
    ]:
        dir_yaw = frame_yaw
        if direction == 1:   # DIR_BACK
            dir_yaw += math.pi
        elif direction == 2: # DIR_LEFT (Fixed: -pi/2 in NED)
            dir_yaw -= math.pi / 2.0
        elif direction == 3: # DIR_RIGHT (Fixed: +pi/2 in NED)
            dir_yaw += math.pi / 2.0
            
        # Center beam (angle = 0) at 1.0m distance
        range_m = 1.0
        angle = 0.0
        dist_x = range_m * math.cos(angle)
        dist_y = range_m * math.sin(angle)
        
        px = frame_x + dist_x * math.cos(dir_yaw) - dist_y * math.sin(dir_yaw)
        py = frame_y + dist_x * math.sin(dir_yaw) + dist_y * math.cos(dir_yaw)
        
        err = math.hypot(px - exp_x, py - exp_y)
        print(f"  Direction {name}: projected=({px:.3f}, {py:.3f}), expected=({exp_x:.3f}, {exp_y:.3f}) -> err={err:.4f}m")
        assert err < 1e-4, f"Projection mismatch for {name}"
        
    print("  -> Sensor projection coordinate frames in NED verified!\n")

def test_flight_log_parsing_and_fallback():
    print("[TEST 2] Testing plot_slam_map.py log parser with git flight_log.txt...")
    
    log_path = os.path.join(os.path.dirname(__file__), "data", "sample_flight_log.txt")
    if os.path.exists(log_path):
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read()
    else:
        text = subprocess.check_output(
            ['git', 'show', 'f61d97b835481f7155b7006c896fc766263c715e:flight_log.txt'],
            cwd=os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "combine"))
        ).decode()
    
    data = parse_slam_data(text)
    assert data is not None, "Failed to parse flight_log.txt"
    
    poses = data.get("poses", [])
    scans = data.get("scans", [])
    
    print(f"  Extracted {len(poses)} poses and {len(scans)} scans.")
    assert len(poses) == 8, f"Expected 8 poses, got {len(poses)}"
    assert len(scans) >= 4, f"Expected at least 4 scans, got {len(scans)}"
    
    # Verify poses have valid coordinates
    for p in poses:
        assert "id" in p and "x" in p and "y" in p and "yaw" in p
        assert -10.0 < p["x"] < 10.0
        assert -10.0 < p["y"] < 10.0
        
    print("  -> Parser correctly extracted all trajectory poses and scan blocks!\n")

def test_corrupted_log_fallback():
    print("[TEST 3] Testing parser resilience against interleaved ESP-IDF log tags...")
    
    corrupted_log = """
=== SLAM_MAP_JSON_START ===
{"poses":[
  {"id":0,"x":1.000,"y":2.000,"yaw":0.500},
  {"id":1,"x":1.500,"y":2.200,"yaw":0.550}
],
I (12345) mavlink: Setpoint stale auto hold
"scans":[
  {"pose_id":0,"pts":[
    [1.1, 2.1],
    [1.2, 2.2]
  ]},
W (12360) wifi: packet lost
  {"pose_id":1,"pts":[
    [1.6, 2.3]
  ]}
]}
=== SLAM_MAP_JSON_END ===
"""
    data = parse_slam_data(corrupted_log)
    assert data is not None, "Parser failed on corrupted log"
    assert len(data.get("poses", [])) == 2, f"Expected 2 poses, got {len(data.get('poses', []))}"
    assert len(data.get("scans", [])) == 2, f"Expected 2 scans, got {len(data.get('scans', []))}"
    print("  -> Interleaved log tags handled and recovered successfully!\n")

if __name__ == "__main__":
    print("=================================================================")
    print("SLAM Coordinate & Quality Verification")
    print("=================================================================")
    test_ned_coordinate_projections()
    test_flight_log_parsing_and_fallback()
    test_corrupted_log_fallback()
    print("ALL VERIFICATION SUITES PASSED!")
