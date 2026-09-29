#!/usr/bin/env python3
"""
High-Speed Vectorized Correlative 2D LiDAR Matcher for HospoBot.
Evaluates the live 2D LaserScan across all candidate rooms, corridors, and 
dense free-space grid locations (1,900+ positions x 72 orientations) against the 
precomputed distance transform in <500ms.
"""

import os
import json
import yaml
import math
import numpy as np

class CorrelativeScanMatcher:
    def __init__(self,
                 map_yaml_path='/home/hospobot/hospobot_ws/global_maps/Building6-Floor1.yaml',
                 rooms_json_path='/home/hospobot/hospobot_ws/web_dash/rooms.json',
                 dist_map_cache='/home/hospobot/hospobot_ws/global_maps/Building6-Floor1_dist.npy'):
        self.map_yaml_path = map_yaml_path
        self.rooms_json_path = rooms_json_path
        self.dist_map_cache = dist_map_cache
        
        self.rooms = {}
        self.origin = np.array([0.0, 0.0])
        self.resolution = 0.05
        self.dist_map = None
        self.h = 0
        self.w = 0
        self.candidates = np.empty((0, 2), dtype=np.float32)
        self.cand_px = np.empty((0, 2), dtype=np.int32)
        self.labels = []
        
        self._load_map()
        self._load_candidates()

    def _load_map(self):
        try:
            if os.path.exists(self.map_yaml_path):
                with open(self.map_yaml_path, 'r') as f:
                    info = yaml.safe_load(f)
                self.origin = np.array(info.get('origin', [0.0, 0.0])[:2])
                self.resolution = float(info.get('resolution', 0.05))
            
            if os.path.exists(self.dist_map_cache):
                self.dist_map = np.load(self.dist_map_cache)
                self.h, self.w = self.dist_map.shape
            else:
                pgm_path = self.map_yaml_path.replace('.yaml', '.pgm')
                from PIL import Image
                from scipy.ndimage import distance_transform_edt
                pgm = np.array(Image.open(pgm_path))
                walls = (pgm == 0)
                self.dist_map = (distance_transform_edt(~walls) * self.resolution).astype(np.float32)
                self.h, self.w = self.dist_map.shape
                try:
                    np.save(self.dist_map_cache, self.dist_map)
                except Exception:
                    pass
        except Exception as e:
            print(f"[ScanMatcher] Error loading map: {e}")

    def _load_candidates(self):
        try:
            if os.path.exists(self.rooms_json_path):
                with open(self.rooms_json_path, 'r') as f:
                    self.rooms = json.load(f)
            
            cands = []
            lbls = []
            
            # 1. All rooms from rooms.json
            for name, r in self.rooms.items():
                cands.append([float(r['x']), float(r['y'])])
                lbls.append(f"Room {name}")
                
            # 2. Corridor interpolation between hallway rooms
            hallway_rooms = ['6-101', '6-109', '6-108', '6-1S1', '6PC1', '6-111', '6-112', '6-113', '6-114', '6-115']
            existing_hallway = [hr for hr in hallway_rooms if hr in self.rooms]
            for i in range(len(existing_hallway) - 1):
                r1 = self.rooms[existing_hallway[i]]
                r2 = self.rooms[existing_hallway[i+1]]
                dist = math.hypot(r2['x'] - r1['x'], r2['y'] - r1['y'])
                n_steps = max(1, int(dist / 1.0))
                for s in range(1, n_steps):
                    frac = s / n_steps
                    cx = r1['x'] + frac * (r2['x'] - r1['x'])
                    cy = r1['y'] + frac * (r2['y'] - r1['y'])
                    cands.append([cx, cy])
                    lbls.append(f"Corridor {existing_hallway[i]}->{existing_hallway[i+1]}")

            # 3. Dense free-space grid search across entire floor (every 0.75m in navigable space)
            if self.dist_map is not None:
                grid_step_px = max(1, int(round(0.75 / self.resolution)))
                py_grid, px_grid = np.mgrid[grid_step_px:self.h:grid_step_px, grid_step_px:self.w:grid_step_px]
                valid_mask = self.dist_map[py_grid, px_grid] > 0.35
                dense_px = px_grid[valid_mask]
                dense_py = py_grid[valid_mask]
                
                my = self.h - 1 - dense_py
                grid_wx = self.origin[0] + dense_px * self.resolution
                grid_wy = self.origin[1] + my * self.resolution
                
                for gx, gy in zip(grid_wx, grid_wy):
                    cands.append([gx, gy])
                    lbls.append(f"Grid ({gx:.1f}, {gy:.1f})")

            self.candidates = np.array(cands, dtype=np.float32)
            self.labels = lbls
            
            # Precompute candidate pixel coordinates
            mx = np.round((self.candidates[:, 0] - self.origin[0]) / self.resolution).astype(np.int32)
            my = np.round((self.candidates[:, 1] - self.origin[1]) / self.resolution).astype(np.int32)
            py = self.h - 1 - my
            self.cand_px = np.column_stack([mx, py])
            
            print(f"[ScanMatcher] Loaded {len(self.candidates)} candidate positions across building floor.")
        except Exception as e:
            print(f"[ScanMatcher] Error loading candidates: {e}")

    def world_to_pixel(self, wx, wy):
        mx = int(round((wx - self.origin[0]) / self.resolution))
        my = int(round((wy - self.origin[1]) / self.resolution))
        return mx, self.h - 1 - my

    def pixel_to_world(self, px, py):
        my = self.h - 1 - py
        return self.origin[0] + px * self.resolution, self.origin[1] + my * self.resolution

    def match_laserscan(self, scan_msg, is_lidar_backward=True):
        """
        Takes a sensor_msgs/LaserScan message.
        `is_lidar_backward`: True if the LiDAR frame is mounted 180° backward (standard HospoBot URDF).
        Returns: (best_x, best_y, best_yaw, score, label) or None
        """
        if self.dist_map is None or len(self.candidates) == 0 or scan_msg is None:
            return None

        try:
            ranges = np.array(scan_msg.ranges, dtype=np.float32)
            n = len(ranges)
            angles = scan_msg.angle_min + np.arange(n) * scan_msg.angle_increment
            
            if is_lidar_backward:
                angles = angles + math.pi  # Convert laser_frame rays to base_link robot frame

            valid = (~np.isnan(ranges)) & (~np.isinf(ranges)) & (ranges > 0.22) & (ranges < 11.5)
            if np.sum(valid) < 15:
                print(f"[ScanMatcher] Insufficient valid scan beams ({np.sum(valid)})")
                return None

            # Sample up to 140 beams evenly across valid angles for high speed
            valid_idx = np.where(valid)[0]
            if len(valid_idx) > 140:
                step = len(valid_idx) // 140
                valid_idx = valid_idx[::step]

            v_ranges = ranges[valid_idx]
            v_angles = angles[valid_idx]
            local_x = v_ranges * np.cos(v_angles)
            local_y = v_ranges * np.sin(v_angles)

            # Stage 1: Global Coarse Search across 72 orientations (every 5 deg)
            test_yaws = np.linspace(-np.pi, np.pi, 72, endpoint=False)
            c_px = self.cand_px[:, 0]
            c_py = self.cand_px[:, 1]
            
            global_top = []

            for yaw in test_yaws:
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                dx = local_x * cos_y - local_y * sin_y
                dy = local_x * sin_y + local_y * cos_y
                dpx = dx / self.resolution
                dpy = -dy / self.resolution
                
                # Vectorized point transformation
                pts_x = np.round(c_px[:, None] + dpx[None, :]).astype(int)
                pts_y = np.round(c_py[:, None] + dpy[None, :]).astype(int)
                
                in_b = (pts_x >= 0) & (pts_x < self.w) & (pts_y >= 0) & (pts_y < self.h)
                pts_x_clp = np.clip(pts_x, 0, self.w - 1)
                pts_y_clp = np.clip(pts_y, 0, self.h - 1)
                
                # Widen global basin of attraction (sigma=0.40m) to tolerate coarse grid
                dists = self.dist_map[pts_y_clp, pts_x_clp]
                scores = np.sum(np.exp(-0.5 * (dists / 0.40)**2) * in_b, axis=1)
                valid_counts = np.sum(in_b, axis=1)
                scores[valid_counts < len(dpx) * 0.55] = -1e9
                
                max_idx = np.argmax(scores)
                max_val = scores[max_idx]
                global_top.append((max_val, int(max_idx), yaw))

            # Keep Top 5 candidates for fine alignment
            global_top.sort(key=lambda x: x[0], reverse=True)
            top_k = min(5, len(global_top))
            global_top = global_top[:top_k]

            best_overall_score = -1e9
            best_overall_res = None

            # Stage 2: Local Sub-Grid Fine Alignment (dx/dy in [-0.7m, +0.7m] @ 0.05m, dyaw in [-10°, +10°] @ 1.5°)
            try:
                for g_score, c_idx, g_yaw in global_top:
                    bx, by = self.candidates[c_idx]
                    label = self.labels[c_idx]
                    
                    fine_dx = np.arange(-0.70, 0.71, 0.05, dtype=np.float32)
                    fine_dy = np.arange(-0.70, 0.71, 0.05, dtype=np.float32)
                    fine_yaws = g_yaw + np.radians(np.arange(-10.0, 10.1, 1.5, dtype=np.float32))

                    fgx, fgy = np.meshgrid(bx + fine_dx, by + fine_dy)
                    fine_x = fgx.ravel()
                    fine_y = fgy.ravel()

                    fine_mx = np.round((fine_x - self.origin[0]) / self.resolution).astype(np.int32)
                    fine_my = np.round((fine_y - self.origin[1]) / self.resolution).astype(np.int32)
                    f_px = fine_mx
                    f_py = self.h - 1 - fine_my

                    fine_best_score = -1e9
                    fine_best_idx = 0
                    fine_best_yaw = g_yaw

                    for fyaw in fine_yaws:
                        cos_y = math.cos(fyaw)
                        sin_y = math.sin(fyaw)
                        dx = local_x * cos_y - local_y * sin_y
                        dy = local_x * sin_y + local_y * cos_y
                        dpx = dx / self.resolution
                        dpy = -dy / self.resolution

                        pts_x = np.round(f_px[:, None] + dpx[None, :]).astype(int)
                        pts_y = np.round(f_py[:, None] + dpy[None, :]).astype(int)

                        in_b = (pts_x >= 0) & (pts_x < self.w) & (pts_y >= 0) & (pts_y < self.h)
                        pts_x_clp = np.clip(pts_x, 0, self.w - 1)
                        pts_y_clp = np.clip(pts_y, 0, self.h - 1)

                        # Sharp basin for fine alignment (sigma=0.15m)
                        dists = self.dist_map[pts_y_clp, pts_x_clp]
                        scores = np.sum(np.exp(-0.5 * (dists / 0.15)**2) * in_b, axis=1)

                        max_idx = np.argmax(scores)
                        max_val = scores[max_idx]
                        if max_val > fine_best_score:
                            fine_best_score = max_val
                            fine_best_idx = int(max_idx)
                            fine_best_yaw = fyaw

                    if fine_best_score > best_overall_score:
                        best_overall_score = float(fine_best_score)
                        best_overall_res = (float(fine_x[fine_best_idx]), float(fine_y[fine_best_idx]), float(fine_best_yaw), best_overall_score, label)
            except Exception as e_fine:
                print(f"[ScanMatcher] Fine refinement warning: {e_fine}")

            return best_overall_res
        except Exception as e:
            print(f"[ScanMatcher] Error in match_laserscan: {e}")
            return None
