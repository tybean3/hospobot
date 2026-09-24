#!/usr/bin/env python3
from PIL import Image
import numpy as np

kp = np.array(Image.open('/home/hospobot/hospobot_ws/global_maps/Building6-Floor1_keepout.pgm'))
m = np.array(Image.open('/home/hospobot/hospobot_ws/global_maps/Building6-Floor1.pgm'))

# Free space in map is 254
# Keepout barriers in keepout.pgm are < 200 (typically 0)
overlap = (m == 254) & (kp < 200)
print(f"Total keepout barrier pixels directly on FREE SPACE: {np.sum(overlap)}")

# Save an image highlighting keepout barriers on the map
rgb = np.stack([m, m, m], axis=-1)
# Green for free space with keepout
rgb[overlap] = [255, 0, 0] # RED for keepout barriers on free space
Image.fromarray(rgb).save('/home/hospobot/hospobot_ws/scratch_keepout_overlay.png')
print("Saved /home/hospobot/hospobot_ws/scratch_keepout_overlay.png")
