#!/usr/bin/env python3
from PIL import Image
import numpy as np

img = Image.open('/home/hospobot/hospobot_ws/scratch_keepout_overlay.png')
# Let's find regions where red pixels exist
arr = np.array(img)
red_mask = (arr[:, :, 0] == 255) & (arr[:, :, 1] == 0) & (arr[:, :, 2] == 0)
y_idx, x_idx = np.where(red_mask)

# Find bounding boxes of clusters
print(f"Red pixels: count={len(y_idx)}, X=[{x_idx.min()}, {x_idx.max()}], Y=[{y_idx.min()}, {y_idx.max()}]")

# Let's find the corridor: the long diagonal or vertical corridor with comb teeth
# In Building6-Floor1_keepout_guide.png, the long corridor runs diagonally from top-left to bottom-right!
# Let's rotate the overlay by -45 degrees (or 45 degrees) to make the corridor vertical like in the user's screenshot!
rot = img.rotate(45, expand=True)
rot.save('/home/hospobot/hospobot_ws/scratch_keepout_rot45.png')
print("Saved scratch_keepout_rot45.png")
