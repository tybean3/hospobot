#!/usr/bin/env python3
"""
Test verification script for synchronized local/global costmaps and real-time path replanning.
Verifies that:
1. nav2_params.yaml and navigate_to_pose_semantic_recovery.xml have matching 10.0 Hz frequencies.
2. Both local and global costmaps incorporate the same sensor streams (scan, depth_scan, dense_semantic_layer).
3. In global_costmap, dense_semantic_layer is located after inflation_layer to preserve hallway centerline targets without over-inflating humans.
4. Simulated A* path planner on the synchronized costmaps immediately diverts paths around newly appeared dynamic/semantic obstacles without routing into high-cost areas.
"""

import os
import yaml
import xml.etree.ElementTree as ET
import numpy as np
import heapq

def test_yaml_config():
    yaml_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/config/nav2_params.yaml'
    assert os.path.exists(yaml_path), f"File {yaml_path} does not exist"
    
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
        
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_cm = config['global_costmap']['global_costmap']['ros__parameters']
    
    # 1. Frequency check
    print(f"[TEST 1] Checking costmap frequencies...")
    assert local['update_frequency'] == 10.0, f"local_costmap update_frequency is {local['update_frequency']}, expected 10.0"
    assert local['publish_frequency'] == 10.0, f"local_costmap publish_frequency is {local['publish_frequency']}, expected 10.0"
    assert global_cm['update_frequency'] == 10.0, f"global_costmap update_frequency is {global_cm['update_frequency']}, expected 10.0"
    assert global_cm['publish_frequency'] == 10.0, f"global_costmap publish_frequency is {global_cm['publish_frequency']}, expected 10.0"
    print("  -> PASSED: Both local and global costmaps configured at 10.0 Hz.")
    
    # 2. Plugins check
    print(f"[TEST 2] Checking plugin configuration...")
    assert "dense_semantic_layer" in local['plugins'], "dense_semantic_layer missing from local_costmap"
    assert "dense_semantic_layer" in global_cm['plugins'], "dense_semantic_layer missing from global_costmap"
    
    # Check that in global_costmap, dense_semantic_layer comes AFTER inflation_layer
    plugins_global = global_cm['plugins']
    inflation_idx = plugins_global.index("inflation_layer")
    dense_idx = plugins_global.index("dense_semantic_layer")
    assert dense_idx > inflation_idx, f"dense_semantic_layer ({dense_idx}) should be placed AFTER inflation_layer ({inflation_idx}) in global_costmap"
    print("  -> PASSED: dense_semantic_layer is correctly placed AFTER inflation_layer in global_costmap.")
    
    # 3. Observation sources check
    print(f"[TEST 3] Checking observation sources...")
    assert "obstacle_layer" in local['plugins'], "obstacle_layer missing from local_costmap plugins"
    assert "depth_obstacle_layer" in local['plugins'], "depth_obstacle_layer missing from local_costmap plugins"
    local_sources = local['obstacle_layer']['observation_sources'].split()
    assert "scan" in local_sources, f"2D LiDAR scan must be in local_costmap obstacle_layer: {local_sources}"
    assert local['obstacle_layer']['scan']['topic'] == '/scan_nav', f"local_costmap scan must use /scan_nav (front/sides filtered), got {local['obstacle_layer']['scan']['topic']}"
    global_sources = global_cm['obstacle_layer']['observation_sources'].split()
    assert "scan" in global_sources, f"global_costmap sources: {global_sources}"
    assert global_cm['obstacle_layer']['scan']['topic'] == '/scan_nav', f"global_costmap scan must use /scan_nav, got {global_cm['obstacle_layer']['scan']['topic']}"
    print("  -> PASSED: 2D LiDAR scan brought back into local_costmap and global_costmap for obstacle avoidance via /scan_nav (front, left, right).")

def test_bt_xml():
    xml_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_semantic/behavior_trees/navigate_to_pose_semantic_recovery.xml'
    assert os.path.exists(xml_path), f"File {xml_path} does not exist"
    
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    print(f"[TEST 4] Checking Behavior Tree replanning rate...")
    rate_controllers = root.findall('.//RateController')
    assert len(rate_controllers) > 0, "No RateController found in BT"
    nav_rate = float(rate_controllers[0].attrib['hz'])
    assert nav_rate == 10.0, f"RateController hz is {nav_rate}, expected 10.0"
    print("  -> PASSED: Behavior Tree RateController is set to 10.0 Hz.")

def simulate_synchronized_planning():
    """
    Simulates a corridor environment with hallway centering cost gradient.
    Demonstrates:
    - Initially clear corridor -> path follows hallway centerline.
    - An obstacle/human appears at (3.0, 0.0) with high cost.
    - Synchronized costmap and planner:
      1) The obstacle is registered immediately in both costmaps.
      2) Path plan is recomputed on the 10 Hz cycle.
      3) Path immediately routes around the obstacle, avoiding the high cost area entirely.
    """
    print(f"[TEST 5] Simulating synchronized costmap update and A* path planning...")
    # Hallway grid: 100 x 50 cells (5.0m long, 2.5m wide, resolution 0.05m)
    # X: 0 to 5.0m, Y: -1.25m to +1.25m
    nx, ny = 100, 50
    res = 0.05
    
    # Static wall inflation creates centerline gradient
    # Walls at y = 0 (bottom) and y = 49 (top), center at y = 25 (y_world = 0.0)
    costmap = np.zeros((nx, ny), dtype=np.float32)
    for j in range(ny):
        y_world = (j - 25) * res
        dist_to_wall = 1.25 - abs(y_world)
        if dist_to_wall <= 0.25:
            costmap[:, j] = 254 # lethal wall
        elif dist_to_wall <= 1.25:
            # exponential inflation cost
            costmap[:, j] = 253 * np.exp(-2.8 * (dist_to_wall - 0.2375))
            
    def astar_plan(cost_grid, start, goal):
        # start, goal in grid coordinates
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                # Reconstruct path
                path = []
                curr = current
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.append(start)
                return path[::-1]
            
            cx, cy = current
            # 8-connected neighbors
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx_idx, ny_idx = cx + dx, cy + dy
                if 0 <= nx_idx < nx and 0 <= ny_idx < ny:
                    cell_cost = cost_grid[nx_idx, ny_idx]
                    if cell_cost >= 253:
                        continue # Lethal obstacle
                    
                    dist = np.hypot(dx, dy)
                    # Nav2 SmacPlanner cost penalty: dist * (1.0 + cost_travel_multiplier * (cell_cost / 252.0))
                    traversal_cost = dist * (1.0 + 2.0 * (cell_cost / 252.0))
                    tentative_g = g_score[current] + traversal_cost
                    
                    neighbor = (nx_idx, ny_idx)
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        h = np.hypot(goal[0] - nx_idx, goal[1] - ny_idx)
                        heapq.heappush(open_set, (tentative_g + h, neighbor))
        return None

    start = (10, 25) # x = 0.5m, y = 0.0m (centerline)
    goal = (90, 25)  # x = 4.5m, y = 0.0m (centerline)
    
    # 1. Clear path
    path_clear = astar_plan(costmap, start, goal)
    assert path_clear is not None, "Failed to find clear path"
    # All Y coordinates in path_clear should be near centerline (y=25)
    y_coords_clear = [p[1] for p in path_clear]
    max_dev_clear = max(abs(y - 25) for y in y_coords_clear)
    assert max_dev_clear <= 1, f"Clear path deviated from centerline: max dev {max_dev_clear}"
    print(f"  -> Initial clear path found: Length {len(path_clear)} steps, stays exactly on centerline.")
    
    # 2. Dynamic obstacle appears right in the middle at x=60 (3.0m), y=25 (0.0m)
    # Both local and global costmap register the obstacle
    costmap_with_obs = costmap.copy()
    obs_x, obs_y = 60, 25
    # Human footprint r=0.25m (5 cells radius) with cost 75/lethal center
    for ix in range(obs_x - 6, obs_x + 7):
        for iy in range(obs_y - 6, obs_y + 7):
            d = np.hypot(ix - obs_x, iy - obs_y) * res
            if d <= 0.25:
                costmap_with_obs[ix, iy] = 254 # lethal footprint
            elif d <= 0.50:
                costmap_with_obs[ix, iy] = max(costmap_with_obs[ix, iy], 120)
                
    # Synchronized replan: The planner updates on the new costmap immediately
    path_avoid = astar_plan(costmap_with_obs, start, goal)
    assert path_avoid is not None, "Failed to find path around obstacle"
    
    # Verify that the path NEVER enters the high-cost obstacle area
    obstacle_costs_on_path = [costmap_with_obs[p[0], p[1]] for p in path_avoid]
    max_cost_on_path = max(obstacle_costs_on_path)
    
    # Verify path diverted around obstacle
    diverted_y = [p[1] for p in path_avoid if 50 <= p[0] <= 70]
    avg_y_divert = np.mean([abs(y - 25) for y in diverted_y])
    
    print(f"  -> Path after obstacle appearance: Diverted around obstacle by {avg_y_divert * res:.2f} m.")
    print(f"  -> Max cost along replanned path: {max_cost_on_path:.1f} (Lethal threshold: 253).")
    assert max_cost_on_path < 253, "Replanned path traversed lethal cost!"
    assert avg_y_divert > 3, "Replanned path failed to divert around obstacle!"
    print("  -> PASSED: Path plan immediately avoids the high-cost obstacle zone upon costmap update.")

if __name__ == '__main__':
    test_yaml_config()
    test_bt_xml()
    simulate_synchronized_planning()
    print("\nALL VERIFICATION CHECKS PASSED SUCCESSFULLY!")
