import os
import random
import math

SDF_HEADER = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="hospital_ward">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics" />
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>0 0 -0.05 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>100 100 0.1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>100 100 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

SDF_FOOTER = """  </world>
</sdf>
"""

def gen_box(name, pose, size, color):
    r, g, b = color
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} {pose[3]:.3f} {pose[4]:.3f} {pose[5]:.3f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

def gen_cyl(name, pose, radius, length, color):
    r, g, b = color
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} {pose[3]:.3f} {pose[4]:.3f} {pose[5]:.3f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius:.3f}</radius><length>{length:.3f}</length></cylinder></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

COLOR_WALL = (0.9, 0.9, 0.9)
COLOR_DOOR = (0.6, 0.4, 0.2)
COLOR_BED = (0.8, 0.8, 0.9)
COLOR_BED_FRAME = (0.5, 0.5, 0.5)
COLOR_TABLE = (0.7, 0.7, 0.7)
COLOR_NURSES = (0.3, 0.3, 0.4)

def generate():
    sdf = [SDF_HEADER]
    
    # 30m Corridor, width 4m (Y=-2 to Y=2).
    # Walls at Y=2 and Y=-2.
    
    room_length = 5.0
    room_depth = 4.0
    wall_h = 3.0
    wall_w = 0.2
    
    # Corridor ends
    sdf.append(gen_box("wall_end_1", (-15.1, 0, wall_h/2, 0, 0, 0), (wall_w, 4.0, wall_h), COLOR_WALL))
    sdf.append(gen_box("wall_end_2", (15.1, 0, wall_h/2, 0, 0, 0), (wall_w, 4.0, wall_h), COLOR_WALL))
    
    # Create rooms
    room_idx = 0
    for side in [1, -1]:
        y_start = 2.0 * side
        y_end = (2.0 + room_depth) * side
        
        # Outer boundary wall
        sdf.append(gen_box(f"outer_wall_{side}", (0, y_end + (wall_w/2)*side, wall_h/2, 0, 0, 0), (30.0, wall_w, wall_h), COLOR_WALL))
        
        for i in range(6):
            x_center = -12.5 + i * room_length
            room_idx += 1
            
            # Divider walls (between rooms)
            sdf.append(gen_box(f"div_wall_{room_idx}", (x_center - room_length/2, y_start + (room_depth/2)*side, wall_h/2, 0, 0, 0), (wall_w, room_depth, wall_h), COLOR_WALL))
            
            # Corridor wall (with door gap)
            # Door width: 1.2m
            door_w = 1.2
            wall_seg_len = room_length - door_w
            sdf.append(gen_box(f"corr_wall_{room_idx}", (x_center - door_w/2, y_start + (wall_w/2)*side, wall_h/2, 0, 0, 0), (wall_seg_len, wall_w, wall_h), COLOR_WALL))
            
            # Door (Random angle)
            door_angle = random.choice([0.0, math.pi/4, math.pi/2, math.pi*3/4]) * side
            door_x = x_center + wall_seg_len/2 - door_w/2
            door_y = y_start + (wall_w/2)*side
            sdf.append(gen_box(f"door_{room_idx}", (door_x + (door_w/2)*math.cos(door_angle), door_y + (door_w/2)*math.sin(door_angle), wall_h/2, 0, 0, door_angle), (door_w, 0.05, 2.2), COLOR_DOOR))
            
            # Bed
            bed_x = x_center + random.uniform(-0.5, 0.5)
            bed_y = y_start + (room_depth/2)*side
            sdf.append(gen_box(f"bed_frame_{room_idx}", (bed_x, bed_y, 0.3, 0, 0, 0), (2.1, 1.1, 0.6), COLOR_BED_FRAME))
            sdf.append(gen_box(f"bed_{room_idx}", (bed_x, bed_y, 0.7, 0, 0, 0), (2.0, 1.0, 0.2), COLOR_BED))
            
            # Bedside table
            sdf.append(gen_box(f"table_{room_idx}", (bed_x - 1.2, bed_y + 0.8*side, 0.4, 0, 0, 0), (0.5, 0.5, 0.8), COLOR_TABLE))
            
            # IV Pole
            sdf.append(gen_cyl(f"iv_pole_{room_idx}", (bed_x + 1.2, bed_y - 0.8*side, 1.0, 0, 0, 0), 0.05, 2.0, (0.8, 0.8, 0.8)))

        # Final divider wall
        sdf.append(gen_box(f"div_wall_end_{side}", (15.0, y_start + (room_depth/2)*side, wall_h/2, 0, 0, 0), (wall_w, room_depth, wall_h), COLOR_WALL))

    # Nurses station
    sdf.append(gen_box("nurse_desk_1", (0, 0, 0.6, 0, 0, 0), (4.0, 1.0, 1.2), COLOR_NURSES))
    sdf.append(gen_box("nurse_desk_2", (-2.5, 0, 0.6, 0, 0, math.pi/4), (2.0, 0.5, 1.2), COLOR_NURSES))
    sdf.append(gen_box("nurse_desk_3", (2.5, 0, 0.6, 0, 0, -math.pi/4), (2.0, 0.5, 1.2), COLOR_NURSES))
    
    # Static obstacles in corridor
    for i in range(5):
        cx = random.uniform(-14, 14)
        cy = random.uniform(-1.5, 1.5)
        # Avoid nurses station
        if abs(cx) < 4.0 and abs(cy) < 1.5:
            cx = 10.0
        
        sdf.append(gen_box(f"cart_{i}", (cx, cy, 0.5, 0, 0, random.uniform(0, math.pi)), (0.8, 0.5, 1.0), (0.5, 0.6, 0.5)))
        sdf.append(gen_cyl(f"person_{i}", (cx+1.0, cy-0.5, 0.85, 0, 0, 0), 0.25, 1.7, (0.8, 0.4, 0.2)))

    sdf.append(SDF_FOOTER)
    
    with open('/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/worlds/hospital.sdf', 'w') as f:
        f.write("".join(sdf))
        
if __name__ == "__main__":
    generate()
    print("Successfully generated detailed hospital ward.")
