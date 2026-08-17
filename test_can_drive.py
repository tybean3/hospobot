import can
import struct
import time

print("Setting up CAN interface...")
bus = can.interface.Bus(channel='can0', bustype='socketcan')

def send_state(node_id, state):
    # CMD_SET_AXIS_STATE = 0x07
    arbitration_id = (node_id << 5) | 0x07
    data = struct.pack('<I', state)
    msg = can.Message(
        arbitration_id=arbitration_id,
        data=list(data),
        is_extended_id=False
    )
    bus.send(msg)

def send_vel(node_id, vel):
    # CMD_SET_INPUT_VEL = 0x0D
    arbitration_id = (node_id << 5) | 0x0D
    data = struct.pack('<ff', vel, 0.0)
    msg = can.Message(
        arbitration_id=arbitration_id,
        data=list(data),
        is_extended_id=False
    )
    bus.send(msg)

print("Setting both motors to Closed Loop Control (State 8)...")
send_state(1, 8)
send_state(2, 8)
time.sleep(1.0)

print("Commanding 1.0 turns/s to Node 1 (S1) and Node 2 (ODESC) for 3 seconds...")
for _ in range(30):
    send_vel(1, 1.0)
    send_vel(2, -1.0) # Opposite direction so robot would go forward
    time.sleep(0.1)

print("Stopping motors...")
for _ in range(5):
    send_vel(1, 0.0)
    send_vel(2, 0.0)
    time.sleep(0.1)

print("Setting both motors to IDLE (State 1)...")
send_state(1, 1)
send_state(2, 1)

print("Done!")
