"""
Tests for the drone-built occupancy grid (main/slam/occ_grid.c) as parsed and
plotted by plot_slam_map.py.

`simulate_occ_grid()` is a line-for-line Python mirror of occ_grid.c, so the
grid the drone will send can be previewed from any saved slam_map.json:

    python tests/test_grid_parse.py                 # run the tests
    python tests/test_grid_parse.py --preview [out.png]
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from plot_slam_map import (GRID_FREE, GRID_OCC, GRID_UNKNOWN, decode_grid,  # noqa: E402
                           parse_grid, parse_slam_data)

# Must match main/slam/occ_grid.h
OCC_RES_M = 0.05
OCC_MARGIN_M = 0.20
OCC_MAX_DIM = 400
OCC_MIN_HITS = 2
OCC_MIN_RATIO_PCT = 35


def simulate_occ_grid(poses, scans):
    """Python mirror of occ_grid_build_and_export(); returns the text the drone prints."""
    pose_map = {p["id"]: p for p in poses}
    used = [(pose_map[s["pose_id"]], s["pts"]) for s in scans if s["pose_id"] in pose_map]
    ns = [p["x"] for p, _ in used] + [q[0] for _, pts in used for q in pts]
    es = [p["y"] for p, _ in used] + [q[1] for _, pts in used for q in pts]
    x0, y0 = min(ns) - OCC_MARGIN_M, min(es) - OCC_MARGIN_M
    span_n, span_e = max(ns) + OCC_MARGIN_M - x0, max(es) + OCC_MARGIN_M - y0
    res = OCC_RES_M
    while span_n / res > OCC_MAX_DIM or span_e / res > OCC_MAX_DIM:
        res *= 1.25
    rows, cols = math.ceil(span_n / res), math.ceil(span_e / res)
    hits = np.zeros((rows, cols), int)
    misses = np.zeros((rows, cols), int)
    last = np.zeros((rows, cols), int)

    def cell(n, e):
        return math.floor((n - x0) / res), math.floor((e - y0) / res)

    for i, (p, pts) in enumerate(used):
        pr, pc = cell(p["x"], p["y"])
        for q in pts:
            hr, hc = cell(q[0], q[1])
            # Bresenham, all cells except the end one (raycast_free)
            r, c = pr, pc
            dr, dc = abs(hr - r), abs(hc - c)
            sr, sc = (1 if r < hr else -1), (1 if c < hc else -1)
            err = dr - dc
            while (r, c) != (hr, hc):
                if 0 <= r < rows and 0 <= c < cols:
                    misses[r, c] = min(misses[r, c] + 1, 255)
                e2 = 2 * err
                if e2 > -dc:
                    err -= dc
                    r += sr
                if e2 < dr:
                    err += dr
                    c += sc
            if 0 <= hr < rows and 0 <= hc < cols and last[hr, hc] != i + 1:
                last[hr, hc] = i + 1
                hits[hr, hc] = min(hits[hr, hc] + 1, 255)

    cls = np.full((rows, cols), GRID_UNKNOWN, np.uint8)
    cls[(misses > 0) | (hits > 0)] = GRID_FREE
    cls[(hits >= OCC_MIN_HITS) & (hits * 100 >= OCC_MIN_RATIO_PCT * (hits + misses))] = GRID_OCC
    for r in range(rows):  # drop isolated occupied cells (in place, as in C)
        for c in range(cols):
            if cls[r, c] == GRID_OCC:
                nb = cls[max(r - 1, 0):r + 2, max(c - 1, 0):c + 2]
                if np.sum(nb == GRID_OCC) == 1:
                    cls[r, c] = GRID_FREE

    sym = "ufo"
    lines = ["=== SLAM_GRID_START ===",
             json.dumps({"res": round(res, 4), "x0": round(x0, 3), "y0": round(y0, 3), "rows": rows, "cols": cols})]
    for row in cls:
        out, c = [], 0
        while c < cols:
            run = 1
            while c + run < cols and row[c + run] == row[c]:
                run += 1
            out.append(f"{run}{sym[row[c]]}")
            c += run
        lines.append("".join(out))
    lines.append("=== SLAM_GRID_END ===")
    return "\n".join(lines)


def test_decode_rle():
    text = """=== SLAM_GRID_START ===
{"res":0.05,"x0":-1.0,"y0":2.0,"rows":2,"cols":5}
2u3f
1o1f3o
=== SLAM_GRID_END ==="""
    grid = parse_grid(text)
    cells = decode_grid(grid)
    assert cells.shape == (2, 5)
    assert cells[0].tolist() == [GRID_UNKNOWN] * 2 + [GRID_FREE] * 3
    assert cells[1].tolist() == [GRID_OCC, GRID_FREE, GRID_OCC, GRID_OCC, GRID_OCC]
    assert (grid["x0"], grid["y0"], grid["res"]) == (-1.0, 2.0, 0.05)
    print("[PASS] RLE rows decode to the right cells and header")


def test_log_noise_interleaved():
    text = """I (1200) slam: done
=== SLAM_GRID_START ===
I (1201) occ_grid: {"res":0.1,"x0":0,"y0":0,"rows":2,"cols":3}
3f
W (1202) nav: Setpoint stale
I (1203) occ_grid: 1f2o
=== SLAM_GRID_END ==="""
    cells = decode_grid(parse_grid(text))
    assert cells.tolist() == [[GRID_FREE] * 3, [GRID_FREE, GRID_OCC, GRID_OCC]]
    print("[PASS] ESP-IDF log lines inside the grid block are ignored")


def test_old_log_without_grid():
    text = """=== SLAM_MAP_JSON_START ===
{"poses":[{"id":0,"x":0.0,"y":0.0,"yaw":0.0}],"scans":[{"pose_id":0,"pts":[[1.0,0.0]]}]}
=== SLAM_MAP_JSON_END ==="""
    data = parse_slam_data(text)
    assert "grid" not in data and len(data["scans"]) == 1
    print("[PASS] Logs from older firmware still parse without a grid")


def test_grid_and_json_together():
    text = """=== SLAM_MAP_JSON_START ===
{"poses":[{"id":0,"x":0.0,"y":0.0,"yaw":0.0}],"scans":[]}
=== SLAM_MAP_JSON_END ===
=== SLAM_GRID_START ===
{"res":0.05,"x0":0,"y0":0,"rows":1,"cols":2}
1f1o
=== SLAM_GRID_END ==="""
    data = parse_slam_data(text)
    assert len(data["poses"]) == 1 and data["grid"]["cols"] == 2
    print("[PASS] Poses (points switched off) and grid combine into one dataset")


def test_simulated_room():
    """Drone at the centre of a 2x2 m room seeing all four walls from 8 poses."""
    rng = np.random.default_rng(1)
    poses, scans = [], []
    for i in range(8):
        px, py = rng.uniform(-0.3, 0.3, 2)
        poses.append({"id": i, "x": px, "y": py, "yaw": 0.0})
        pts = []
        for a in np.linspace(0, 2 * np.pi, 64, endpoint=False):
            d = np.array([np.cos(a), np.sin(a)])
            t = min((np.sign(d[k]) - [px, py][k]) / d[k] for k in (0, 1) if abs(d[k]) > 1e-9)
            n = px + t * d[0] + rng.normal(0, 0.02)
            e = py + t * d[1] + rng.normal(0, 0.02)
            pts.append([n, e])
        pts.append([rng.uniform(-0.8, 0.8), rng.uniform(-0.8, 0.8)])  # one stray reflection per scan
        scans.append({"pose_id": i, "pts": pts})
    grid = parse_grid(simulate_occ_grid(poses, scans))
    cells = decode_grid(grid)
    res, x0, y0 = grid["res"], grid["x0"], grid["y0"]
    inner = cells[int((-0.7 - x0) / res):int((0.7 - x0) / res), int((-0.7 - y0) / res):int((0.7 - y0) / res)]
    assert np.sum(inner == GRID_OCC) == 0, "stray one-scan reflections inside the room must be filtered"
    wall_row = int((1.0 - x0) / res)
    assert np.sum(cells[wall_row - 1:wall_row + 2] == GRID_OCC) > 20, "north wall must be occupied"
    print("[PASS] Simulated room: walls occupied, stray reflections removed")


def preview(json_path, out_png):
    import matplotlib
    matplotlib.use("Agg")
    from plot_slam_map import plot_slam_map
    data = json.load(open(json_path))
    data["grid"] = parse_grid(simulate_occ_grid(data["poses"], data["scans"]))
    cells = decode_grid(data["grid"])
    print(f"Grid {cells.shape} @ {data['grid']['res']} m: occupied {np.sum(cells == GRID_OCC)}, "
          f"free {np.sum(cells == GRID_FREE)}, unknown {np.sum(cells == GRID_UNKNOWN)}")
    plot_slam_map(data, output_file=None, show_gui=False, title="Drone grid preview (simulated)")
    import matplotlib.pyplot as plt
    plt.savefig(out_png, dpi=120, facecolor="#1e1e1e")
    print(f"Preview written to {out_png}")


if __name__ == "__main__":
    if "--preview" in sys.argv:
        idx = sys.argv.index("--preview")
        out = sys.argv[idx + 1] if idx + 1 < len(sys.argv) else "grid_preview.png"
        preview(Path(__file__).resolve().parent.parent / "laptop" / "data" / "slam_map.json", out)
        sys.exit(0)
    print("=== Occupancy grid parse/plot tests ===")
    test_decode_rle()
    test_log_noise_interleaved()
    test_old_log_without_grid()
    test_grid_and_json_together()
    test_simulated_room()
    print("All grid tests passed.")
