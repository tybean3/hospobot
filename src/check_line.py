#!/usr/bin/env python3
import yaml
from PIL import Image
import numpy as np

# Load map metadata
with open('/home/hospobot/hospobot_ws/global_maps/Building6-Floor1.yaml') as f:
    info = yaml.safe_load(f)

origin = info['origin'] # [-37.086, -41.989, 0]
res = info['resolution'] # 0.05

m = np.array(Image.open('/home/hospobot/hospobot_ws/global_maps/Building6-Floor1.pgm'))
kp = np.array(Image.open('/home/hospobot/hospobot_ws/global_maps/Building6-Floor1_keepout.pgm'))
h, w = m.shape

def world_to_map(wx, wy):
    mx = int((wx - origin[0]) / res)
    my = int((wy - origin[1]) / res)
    # In PGM image, row 0 is top, which corresponds to my = h - 1 - py
    py = h - 1 - my
    px = mx
    return px, py

start_px, start_py = world_to_map(33.71, -30.62)
goal_px, goal_py = world_to_map(-8.10, 3.84)

print(f"Start world (33.71, -30.62) -> pixel ({start_px}, {start_py})")
print(f"Goal world (-8.10, 3.84) -> pixel ({goal_px}, {goal_py})")

# Step along the line from start to goal and check map and keepout values
steps = 200
for t in np.linspace(0, 1, steps):
    px = int(round(start_px + t * (goal_px - start_px)))
    py = int(round(start_py + t * (goal_py - start_py)))
    if 0 <= px < w and 0 <= py < h:
        mv = m[py, px]
        kpv = kp[py, px]
        wx = origin[0] + px * res
        wy = origin[1] + (h - 1 - py) * res
        if kpv < 200 or mv != 254:
            print(f"Obstacle/Keepout at t={t:.2f} ({wx:.2f}, {wy:.2f}), pixel ({px}, {py}): map={mv}, keepout={kpv}")
