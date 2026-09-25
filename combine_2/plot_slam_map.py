"""
SLAM Map & Trajectory Visualizer + Dataset Exporter
Usage:
    # 1. Fully Automatic Live Mode:
    python plot_slam_map.py --live

    # 2. Instant Plot from Clipboard (Copy log, then run):
    python plot_slam_map.py --clipboard

    # 3. Plot from saved dataset or file:
    python plot_slam_map.py [flight_log.txt]
"""

import os
import sys
import re
import json
import csv
import threading
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

DATA_DIR = Path(__file__).parent / "laptop" / "data"
if not DATA_DIR.parent.exists():
    DATA_DIR = Path(__file__).parent / "data"

def save_dataset_to_laptop(data, data_dir=DATA_DIR):
    """Saves the parsed SLAM poses, scans, and points into laptop/data/"""
    data_dir.mkdir(parents=True, exist_ok=True)
    
    poses = data.get("poses", [])
    scans = data.get("scans", [])

    # 1. Save full JSON
    json_path = data_dir / "slam_map.json"
    with open(json_path, "w") as f:
        json.dump(data, f, indent=2)

    # 2. Save poses.csv (drone_id/node_id, x, y, yaw)
    poses_csv_path = data_dir / "poses.csv"
    with open(poses_csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["node_id", "x_north", "y_east", "yaw_rad"])
        for p in poses:
            writer.writerow([f"0/{p['id']}", p["x"], p["y"], p["yaw"]])

    # 3. Save points.csv (all 2D obstacle coordinates)
    points_csv_path = data_dir / "points.csv"
    with open(points_csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["pose_id", "x_north", "y_east"])
        for sc in scans:
            p_id = sc.get("pose_id", 0)
            for pt in sc.get("pts", []):
                writer.writerow([p_id, pt[0], pt[1]])

    print(f"\n[+] Dataset successfully saved to: {data_dir.resolve()}")
    print(f"    - {json_path.name}")
    print(f"    - {poses_csv_path.name}")
    print(f"    - {points_csv_path.name}")

def parse_raw_poses(text):
    """Pre-optimisation odometry poses printed by slam_bridge_export_json()."""
    match = re.search(r'=== SLAM_RAW_POSES_START ===(.*?)=== SLAM_RAW_POSES_END ===', text or "", re.DOTALL)
    if not match:
        return []
    return [{"id": i, "x": float(m[0]), "y": float(m[1]), "yaw": float(m[2])}
            for i, m in enumerate(re.findall(r'\[\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\]', match.group(1)))]

def parse_slam_data(text):
    data = _parse_slam_map(text)
    if data is not None and "raw_poses" not in data:
        raw = parse_raw_poses(text)
        if raw:
            data["raw_poses"] = raw
    return data

def _parse_slam_map(text):
    if not text:
        return None

    # 1. Extract JSON block between === SLAM_MAP_JSON_START === and === SLAM_MAP_JSON_END ===
    match = re.search(r'=== SLAM_MAP_JSON_START ===(.*?)=== SLAM_MAP_JSON_END ===', text, re.DOTALL)
    if match:
        raw_block = match.group(1).strip()
        cleaned_lines = []
        for line in raw_block.splitlines():
            l_str = line.strip()
            # Strip any ESP-IDF log prefixes if present e.g. I (1234) tag:
            cleaned = re.sub(r'^[IEWDV]\s+\(\d+\)\s+[\w_]+:\s*', '', l_str).strip()
            if not cleaned:
                continue
            # A valid JSON line in this schema starts with JSON structural or numeric chars
            first_char = cleaned[0]
            if first_char not in ['{', '}', '[', ']', '"', ',', '-', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
                continue
            cleaned_lines.append(cleaned)
        cleaned_json = "\n".join(cleaned_lines)
        try:
            return json.loads(cleaned_json)
        except Exception as e:
            print(f"JSON Parse Note: {e}. Falling back to candidate extractor...")

    # 2. Extract from first {"poses" to last }
    start_idx = text.find('{"poses"')
    if start_idx == -1:
        start_idx = text.find('{')
    end_idx = text.rfind('}')
    if start_idx != -1 and end_idx != -1 and end_idx > start_idx:
        candidate = text[start_idx:end_idx + 1]
        cleaned_lines = []
        for line in candidate.splitlines():
            c = re.sub(r'^[IEWDV]\s+\(\d+\)\s+[\w_]+:\s*', '', line.strip())
            if not any(bad in c for bad in ["Setpoint stale", "auto hold", "crosstool-NG", "Free SRAM"]):
                cleaned_lines.append(c)
        try:
            return json.loads("\n".join(cleaned_lines))
        except Exception:
            pass

    # 3. Fallback: Parse poses (JSON or ASCII) & point cloud scans
    poses = []
    json_pose_matches = re.findall(r'\{\s*"id"\s*:\s*(\d+)\s*,\s*"x"\s*:\s*([-\d\.]+)\s*,\s*"y"\s*:\s*([-\d\.]+)\s*,\s*"yaw"\s*:\s*([-\d\.]+)\s*\}', text)
    for m in json_pose_matches:
        poses.append({
            "id": int(m[0]),
            "x": float(m[1]),
            "y": float(m[2]),
            "yaw": float(m[3])
        })

    if not poses:
        pose_matches = re.findall(r'Pose\s+\[(\d+):(\d+)\]\s+->\s+x=([-\d\.]+),\s+y=([-\d\.]+),\s+yaw=([-\d\.]+)', text)
        for m in pose_matches:
            poses.append({
                "id": int(m[1]),
                "x": float(m[2]),
                "y": float(m[3]),
                "yaw": float(m[4])
            })

    scans = []
    scan_blocks = re.findall(r'\{\s*"pose_id"\s*:\s*(\d+)\s*,\s*"pts"\s*:\s*\[(.*?)\]\s*\}', text, re.DOTALL)
    for s_match in scan_blocks:
        p_id = int(s_match[0])
        pts_text = s_match[1]
        pts = []
        for pm in re.findall(r'\[\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\]', pts_text):
            try:
                pts.append([float(pm[0]), float(pm[1])])
            except ValueError:
                continue
        if pts:
            scans.append({"pose_id": p_id, "pts": pts})

    if not scans:
        points = []
        search_target = match.group(1) if match else text
        pt_matches = re.findall(r'\[\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\]', search_target)
        for m in pt_matches:
            try:
                points.append([float(m[0]), float(m[1])])
            except ValueError:
                continue
        if points:
            scans = [{"pose_id": 0, "pts": points}]

    if poses or scans:
        return {"poses": poses, "scans": scans}

    return None

def plot_slam_map(data, output_file="image_test_1.2.png", title="Image Test 1.2 - 2D SLAM Map", show_gui=True):
    poses = data.get("poses", [])
    scans = data.get("scans", [])

    if not poses and not scans:
        print("No SLAM data found to plot.")
        return

    plt.figure(figsize=(9, 9), facecolor="#1e1e1e")
    ax = plt.gca()
    ax.set_facecolor("#121212")

    # 1. Plot 2D obstacle point cloud from ToF scans
    pose_map = {p["id"]: p for p in poses}
    all_pts_x = []
    all_pts_y = []
    free_rays = []
    for sc in scans:
        p_id = sc.get("pose_id", 0)
        p = pose_map.get(p_id)
        for pt in sc.get("pts", []):
            if p:
                dist_to_drone = np.hypot(pt[0] - p["x"], pt[1] - p["y"])
                if dist_to_drone < 0.18:
                    continue  # Filter out points hitting the drone itself (< 0.18 m)
                free_rays.append([(p["y"], p["x"]), (pt[1], pt[0])])
            all_pts_x.append(pt[1]) # X-axis = East
            all_pts_y.append(pt[0]) # Y-axis = North

    # Space each beam passed through before hitting something is known-free;
    # shading it shows the room's open floor, not just the wall hits.
    if free_rays:
        from matplotlib.collections import LineCollection
        ax.add_collection(LineCollection(free_rays, colors="#3a6ea5", linewidths=0.6, alpha=0.08, zorder=2))
        ax.plot([], [], color="#3a6ea5", linewidth=3, alpha=0.6, label="Observed free space")

    if all_pts_x:
        ax.scatter(all_pts_x, all_pts_y, s=16, c="#00ffcc", alpha=0.75, label=f"ToF Obstacle Points ({len(all_pts_x)} pts)", zorder=3)

    # 2. Plot raw odometry trajectory (before LS-SLAM) for comparison
    raw_poses = data.get("raw_poses", [])
    if raw_poses:
        shift = [np.hypot(r["x"] - p["x"], r["y"] - p["y"]) for r, p in zip(raw_poses, poses)]
        ax.plot([r["y"] for r in raw_poses], [r["x"] for r in raw_poses], color="#aaaaaa", linewidth=1.5,
                linestyle="--", marker="x", markersize=5,
                label=f"Raw odometry (SLAM shift max {max(shift, default=0):.2f} m)", zorder=4)

    # 3. Plot Trajectory & Heading
    if poses:
        xs = [p["y"] for p in poses] # East
        ys = [p["x"] for p in poses] # North

        ax.plot(xs, ys, color="#ff9900", linewidth=2.2, linestyle="-", marker="o", markersize=6, label="Optimised Trajectory", zorder=4)
        ax.scatter([xs[0]], [ys[0]], s=120, c="#00ff00", edgecolors="white", label="Takeoff [0:0]", zorder=6)
        ax.scatter([xs[-1]], [ys[-1]], s=120, c="#ff3333", edgecolors="white", label=f"Landing [0:{poses[-1]['id']}]", zorder=6)

        for p in poses:
            yaw = p["yaw"]
            dx = 0.25 * np.sin(yaw)
            dy = 0.25 * np.cos(yaw)
            ax.arrow(p["y"], p["x"], dx, dy, head_width=0.08, head_length=0.08, fc="#ffcc00", ec="#ffcc00", zorder=5)

    ax.set_title(title, color="white", fontsize=14, fontweight="bold", pad=15)
    ax.set_xlabel("East (metres)", color="white", fontsize=11)
    ax.set_ylabel("North (metres)", color="white", fontsize=11)
    ax.tick_params(colors="white")
    ax.grid(True, linestyle="--", color="#333333", alpha=0.7)
    ax.axis("equal")
    ax.legend(facecolor="#222222", edgecolor="#444444", labelcolor="white", loc="upper left")

    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=150, facecolor="#1e1e1e")
        laptop_img_path = DATA_DIR / Path(output_file).name
        plt.savefig(laptop_img_path, dpi=150, facecolor="#1e1e1e")
        print(f"[+] Map image saved to: {output_file} & {laptop_img_path}")
        if sys.platform == "win32" and os.path.exists(output_file):
            try:
                os.startfile(output_file)
            except Exception:
                pass

    if show_gui:
        try:
            plt.show()
        except Exception:
            pass

def listen_serial(port="COM4", baudrate=115200, output_file="image_test_1.2.png", show_gui=True):
    """Listens live to the ESP32 on COM port and automatically plots & saves upon landing."""
    import serial
    print(f"Connecting to ESP32 on {port} @ {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1.0)
    except Exception as e:
        print(f"Error opening serial port {port}: {e}")
        print(f"Tip: If another terminal/monitor is using {port}, close it and retry.")
        return
    import time
    try:
        ser.dtr = False
        ser.rts = False
        time.sleep(0.05)
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
    except Exception:
        pass

    print(f"Connected to {port}! (Streaming live... press Enter at any time to generate plot from current data)\n")
    buffer = []
    capturing_json = False
    json_lines = []

    import queue
    plot_queue = queue.Queue()

    def handle_line(line):
        nonlocal capturing_json
        print(line, end="")
        if "=== SLAM_MAP_JSON_START ===" in line:
            capturing_json = True
            json_lines.clear()
            json_lines.append(line)
        elif "=== SLAM_MAP_JSON_END ===" in line:
            capturing_json = False
            json_lines.append(line)
            full_text = "".join(json_lines)
            data = parse_slam_data(full_text)
            if data:
                raw = parse_raw_poses("".join(buffer))
                if raw:
                    data["raw_poses"] = raw
                print("\n[!] Received complete SLAM dataset from drone!")
                save_dataset_to_laptop(data)
                plot_queue.put(data)
        elif capturing_json:
            json_lines.append(line)

    def read_loop():
        while True:
            try:
                line = ser.readline().decode("utf-8", errors="replace")
                if line:
                    buffer.append(line)
                    handle_line(line)
            except Exception:
                break

    def input_loop():
        while True:
            try:
                _ = input()
                raw_text = "".join(buffer)
                data = parse_slam_data(raw_text)
                if not data and (DATA_DIR / "slam_map.json").exists():
                    try:
                        with open(DATA_DIR / "slam_map.json", "r") as f:
                            data = json.load(f)
                    except Exception:
                        pass
                if data:
                    print("\n[!] Manual trigger: plotting available map data...")
                    plot_queue.put(data)
                else:
                    print("[?] Waiting for SLAM map data from drone... (Tip: Map is generated after drone lands)")
            except (EOFError, KeyboardInterrupt):
                break
            except Exception:
                pass

    t = threading.Thread(target=read_loop, daemon=True)
    t.start()
    t_in = threading.Thread(target=input_loop, daemon=True)
    t_in.start()

    try:
        while t.is_alive():
            try:
                data = plot_queue.get(timeout=0.2)
                # Execute GUI plot on the MAIN thread to avoid Matplotlib thread warnings
                plot_slam_map(data, output_file, title="Image Test 1.2 - 2D SLAM Map", show_gui=show_gui)
                if sys.platform == "win32" and output_file and os.path.exists(output_file):
                    try:
                        os.startfile(output_file)
                    except Exception:
                        pass
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        print("\nDisconnected from serial monitor.")
    finally:
        ser.close()

def get_clipboard_text():
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        clip = root.clipboard_get()
        root.destroy()
        return clip
    except Exception:
        return ""

if __name__ == "__main__":
    output_filename = "image_test_1.2.png"
    show_gui = "--no-gui" not in sys.argv

    port = None
    if "--port" in sys.argv:
        idx = sys.argv.index("--port")
        if idx + 1 < len(sys.argv):
            port = sys.argv[idx + 1]
        else:
            port = "COM4"
    elif "--live" in sys.argv:
        port = "COM4"

    if port:
        listen_serial(port=port, output_file=output_filename, show_gui=show_gui)
        sys.exit(0)

    content = ""
    file_args = [a for a in sys.argv[1:] if not a.startswith("--") and a != output_filename]

    if file_args:
        filename = file_args[0]
        for enc in ["utf-8", "utf-16", "utf-16-le", "latin-1"]:
            try:
                with open(filename, "r", encoding=enc) as f:
                    test_c = f.read()
                    if "SLAM_MAP_JSON" in test_c or "swarm_graph" in test_c or "Pose [" in test_c:
                        content = test_c
                        break
                    elif not content:
                        content = test_c
            except Exception:
                continue
    else:
        # Check clipboard
        clip = get_clipboard_text()
        if "=== SLAM_MAP_JSON_START ===" in clip or "=== SWARM GRAPH" in clip or "Pose [" in clip:
            print("Auto-detected SLAM data in clipboard!")
            content = clip
        elif (DATA_DIR / "slam_map.json").exists():
            print(f"Loading existing dataset from {DATA_DIR / 'slam_map.json'}...")
            with open(DATA_DIR / "slam_map.json", "r") as f:
                content = f.read()
        else:
            print("Paste your terminal log below, then press Ctrl+Z and Enter:\n")
            content = sys.stdin.read()

    data = parse_slam_data(content)
    if data:
        print(f"Extracted {len(data.get('poses', []))} poses and {len(data.get('scans', []))} point cloud scans.")
        save_dataset_to_laptop(data)
        plot_slam_map(data, output_filename, title="Image Test 1.2 - 2D SLAM Map", show_gui=show_gui)
    else:
        print("Could not find SLAM graph output in the provided input.")
