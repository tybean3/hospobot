#!/usr/bin/env python3
"""
Utility script to generate Nav2 Keep-Out Filter Mask templates.
Usage:
    python3 create_keepout_mask.py <map_name_or_path>

Example:
    python3 create_keepout_mask.py wwww
    python3 create_keepout_mask.py /home/hospobot/hospobot_ws/global_maps/wwww.yaml
"""

import sys
import os
import yaml
from PIL import Image

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 create_keepout_mask.py <map_name_or_yaml_path>")
        sys.exit(1)

    arg = sys.argv[1]
    global_maps_dir = "/home/hospobot/hospobot_ws/global_maps"

    if os.path.exists(arg):
        yaml_path = os.path.abspath(arg)
    else:
        # Check in global_maps
        name = arg if not arg.endswith('.yaml') else arg[:-5]
        candidate = os.path.join(global_maps_dir, f"{name}.yaml")
        if os.path.exists(candidate):
            yaml_path = candidate
        else:
            print(f"Error: Could not find map '{arg}' or '{candidate}'")
            sys.exit(1)

    map_dir = os.path.dirname(yaml_path)
    base_name = os.path.splitext(os.path.basename(yaml_path))[0]

    with open(yaml_path, 'r') as f:
        map_meta = yaml.safe_load(f)

    image_rel = map_meta.get('image', f"{base_name}.pgm")
    image_path = os.path.join(map_dir, image_rel)

    if not os.path.exists(image_path):
        print(f"Error: Map image '{image_path}' not found!")
        sys.exit(1)

    print(f"Loading map: {yaml_path}")
    base_img = Image.open(image_path).convert('L')
    width, height = base_img.size
    print(f"Map size: {width} x {height}, resolution: {map_meta.get('resolution', 0.05)}")

    # 1. Create blank white keepout PGM (254 = free space / no restriction)
    # 0 in PGM = 100% cost (Keep-out / Lethal barrier)
    keepout_pgm_name = f"{base_name}_keepout.pgm"
    keepout_pgm_path = os.path.join(map_dir, keepout_pgm_name)
    
    # If keepout pgm already exists, don't overwrite user's drawings unless forced
    if not os.path.exists(keepout_pgm_path):
        keepout_img = Image.new('L', (width, height), 254)
        keepout_img.save(keepout_pgm_path)
        print(f"Created blank keepout mask: {keepout_pgm_path}")
    else:
        print(f"Notice: Existing keepout mask found at {keepout_pgm_path} (preserved)")

    # 2. Create keepout YAML
    keepout_yaml_name = f"{base_name}_keepout.yaml"
    keepout_yaml_path = os.path.join(map_dir, keepout_yaml_name)
    keepout_meta = {
        'image': keepout_pgm_name,
        'mode': 'trinary',
        'resolution': map_meta.get('resolution', 0.05),
        'origin': map_meta.get('origin', [0.0, 0.0, 0.0]),
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    with open(keepout_yaml_path, 'w') as f:
        yaml.dump(keepout_meta, f, default_flow_style=None)
    print(f"Created keepout metadata: {keepout_yaml_path}")

    # 3. Create a guide overlay image (PNG) for easy editing in GIMP / Photoshop
    # This shows the actual building walls faintly in grey, so the user can easily
    # draw black lines (#000000) over glass doors and save as <map>_keepout.pgm!
    guide_png_path = os.path.join(map_dir, f"{base_name}_keepout_guide.png")
    # Faintly blend walls (0 -> 180 light grey) so user sees where walls are
    pixels = base_img.load()
    guide_img = Image.new('L', (width, height), 254)
    guide_pixels = guide_img.load()
    for y in range(height):
        for x in range(width):
            val = pixels[x, y]
            if val < 50:  # Wall obstacle
                guide_pixels[x, y] = 180  # Faint grey outline
            else:
                guide_pixels[x, y] = 254  # White background
    guide_img.save(guide_png_path)
    print(f"Created editing guide image: {guide_png_path}")
    print("\nHow to use:")
    print(f"1. Open '{guide_png_path}' in GIMP / Photoshop.")
    print("2. Use a black brush/pencil (#000000) to draw solid lines across glass doors or restricted areas.")
    print(f"3. Export / Save the result as '{keepout_pgm_path}'.")
    print("4. Done! Nav2 will now strictly block those zones while AMCL localizes against the real walls.")

if __name__ == '__main__':
    main()
