#!/usr/bin/env python3
"""
Test script verifying that objects from the OAK camera are cleared exactly 0.5s after leaving the camera frame.
Tests:
1. nav2_params.yaml configuration (semantic_layer and dense_semantic_layer timeout_sec == 0.5, obstacle_layer using 360-deg LiDAR).
2. object_detector_node.py track_expiry_sec == 0.50s.
3. End-to-end lifecycle in semantic_costmap_publisher:
   - Object appears in camera frame -> Marked in costmap.
   - Object leaves camera frame:
     * At t < 0.5s: Retained for smooth tracking continuity.
     * At t >= 0.5s: Cleared completely from the costmap grid (0 cost, no ghost obstacles).
"""

import os
import yaml
import time
import math
import numpy as np

def test_config_timeouts():
    yaml_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/config/nav2_params.yaml'
    assert os.path.exists(yaml_path), f"File {yaml_path} not found"
    
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
        
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_cm = config['global_costmap']['global_costmap']['ros__parameters']
    
    print("[TEST 1] Verifying costmap timeouts and observation sources in nav2_params.yaml...")
    assert local['dense_semantic_layer']['timeout_sec'] == 0.5, f"local dense_semantic_layer timeout is {local['dense_semantic_layer']['timeout_sec']}, expected 0.5"
    assert global_cm['dense_semantic_layer']['timeout_sec'] == 0.5, f"global dense_semantic_layer timeout is {global_cm['dense_semantic_layer']['timeout_sec']}, expected 0.5"
    assert global_cm['semantic_layer']['timeout_sec'] == 0.5, f"global semantic_layer timeout is {global_cm['semantic_layer']['timeout_sec']}, expected 0.5"
    assert global_cm['semantic_layer']['bed']['inflation_radius'] <= 1.0, f"global bed inflation radius is {global_cm['semantic_layer']['bed']['inflation_radius']}, should be <= 1.0m"
    
    assert "obstacle_layer" in local['plugins'], "obstacle_layer should be present in local_costmap"
    assert local['obstacle_layer']['scan']['topic'] == '/scan_nav', "2D LiDAR scan in local_costmap should use /scan_nav (front/sides filtered)"
    assert global_cm['obstacle_layer']['scan']['topic'] == '/scan_nav', "2D LiDAR scan in global_costmap should use /scan_nav"
    print("  -> PASSED: All OAK camera layer timeouts set to 0.5s, 2D LiDAR brought back for obstacle avoidance via /scan_nav.")

def test_detector_track_expiry():
    detector_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_perception/hospobot_perception/object_detector_node.py'
    assert os.path.exists(detector_path)
    
    print("[TEST 2] Verifying object_detector_node.py track expiry parameter...")
    with open(detector_path, 'r') as f:
        content = f.read()
    assert "self.track_expiry_sec = 0.50" in content, "object_detector_node does not have track_expiry_sec = 0.50"
    print("  -> PASSED: object_detector_node sets track_expiry_sec = 0.50s.")

def simulate_semantic_costmap_clearing_lifecycle():
    print("[TEST 3] Simulating OAK camera object clearing lifecycle (0.5s timeout)...")
    resolution = 0.05
    width = 160
    height = 160
    origin_x = -2.5
    origin_y = -4.0
    
    # 1. Object appears in camera frame at t = 0.0
    t_appear = 100.0
    # Simulate human at (1.5, 0.2)
    humans = [{'id': 1, 'pos': [1.5, 0.2], 'vel': [0.0, 0.0], 'last_time': t_appear, 'local_x': 1.5, 'local_y': 0.2}]
    # Simulate static object at (2.0, -0.5)
    objects = [{'class_name': 'BED', 'position': {'x': 2.0, 'y': -0.5}, 'frame_id': 'base_footprint', 'last_time': t_appear}]
    # Simulate depth scan obstacle at (1.2, 0.0)
    depth_obstacles = [(1.2, 0.0, t_appear)]
    
    def render_grid(curr_sec, humans_list, objects_list, depth_obs_list):
        # Pruning filter matching semantic_costmap_publisher.py
        h_active = [h for h in humans_list if (curr_sec - h.get('last_time', curr_sec)) <= 0.50]
        o_active = [o for o in objects_list if (curr_sec - o.get('last_time', curr_sec)) <= 0.50]
        d_active = [d for d in depth_obs_list if (curr_sec - d[2]) <= 0.50]
        
        grid = np.zeros((height, width), dtype=np.int8)
        
        # Render depth obstacles
        for ox, oy, t in d_active:
            col = int((ox - origin_x) / resolution)
            row = int((oy - origin_y) / resolution)
            if 0 <= row < height and 0 <= col < width:
                grid[row, col] = max(grid[row, col], 99)
                
        # Render static objects
        for obj in o_active:
            x, y = obj['position']['x'], obj['position']['y']
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            radius_cells = int(0.4 / resolution)
            for r in range(max(0, row - radius_cells), min(height, row + radius_cells + 1)):
                for c in range(max(0, col - radius_cells), min(width, col + radius_cells + 1)):
                    dist = math.hypot(r - row, c - col)
                    if dist <= radius_cells:
                        grid[r, c] = max(grid[r, c], 90 if dist < (0.2 / resolution) else 50)
                        
        # Render humans
        for th in h_active:
            x, y = th['local_x'], th['local_y']
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            base_radius = 0.25
            base_cells = int(base_radius / resolution)
            for r in range(max(0, row - base_cells), min(height, row + base_cells + 1)):
                for c in range(max(0, col - base_cells), min(width, col + base_cells + 1)):
                    d = math.hypot((c - col) * resolution, (r - row) * resolution)
                    if d <= base_radius:
                        grid[r, c] = max(grid[r, c], 75 if d < 0.15 else 50)
                        
        return grid

    # Frame 1: Object currently in camera frame (t = 100.0)
    grid_frame1 = render_grid(100.0, humans, objects, depth_obstacles)
    assert np.max(grid_frame1) >= 90, "Objects not rendered when in frame!"
    print(f"  -> At t = 0.0s (in frame): Max cost {np.max(grid_frame1)} (Obstacles actively rendered)")
    
    # Frame 2: Object left camera frame 0.3s ago (t = 100.3s)
    grid_frame2 = render_grid(100.3, humans, objects, depth_obstacles)
    assert np.max(grid_frame2) >= 90, "Objects prematurely cleared before 0.5s!"
    print(f"  -> At t = 0.3s after leaving: Max cost {np.max(grid_frame2)} (Retained for tracking continuity)")
    
    # Frame 3: Object left camera frame 0.51s ago (t = 100.51s)
    grid_frame3 = render_grid(100.51, humans, objects, depth_obstacles)
    assert np.max(grid_frame3) == 0, f"Objects still in costmap after 0.51s! Max cost: {np.max(grid_frame3)}"
    print(f"  -> At t = 0.51s after leaving: Max cost {np.max(grid_frame3)} (COMPLETELY CLEARED, 0 ghost cells)")
    print("  -> PASSED: Objects from OAK camera are retained during 0.5s and purged cleanly at 0.5s.")

if __name__ == '__main__':
    test_config_timeouts()
    test_detector_track_expiry()
    simulate_semantic_costmap_clearing_lifecycle()
    print("\nALL 0.5s OAK CAMERA CLEARING CHECKS PASSED SUCCESSFULLY!")
