import sys
import time
try:
    import odrive
    from odrive.enums import *
except ImportError:
    print("Error: 'odrive' python package not found.")
    sys.exit(1)

print("Searching for ODrive S1 (Left Motor) via USB...")
odrv0 = odrive.find_any(serial_number="395635433231", timeout=10)
if not odrv0:
    print("Could not find ODrive S1. Make sure it is plugged in.")
    sys.exit(1)
print(f"Connected to ODrive S1 (Serial: {odrv0.serial_number})")

print("\nStarting Encoder Slip & Stall Diagnostic Tool")
print("Press CTRL+C to exit.\n")
print(f"{'Time(s)':<8} | {'Cmd Vel (trn/s)':<17} | {'Act Vel (trn/s)':<17} | {'Pos (trn)':<12} | {'Iq Set (A)':<12} | {'Iq Meas (A)':<12}")
print("-" * 85)

start_time = time.time()

try:
    while True:
        t = time.time() - start_time
        
        # Read commanded velocity (what the controller WANTS to do)
        cmd_vel = odrv0.axis0.controller.input_vel
        
        # Read actual estimated velocity and position (what the encoder THINKS is happening)
        # For ODrive S1 (firmware 0.6.x), this is usually in pos_vel_mapper
        try:
            act_vel = odrv0.axis0.vel_estimate
            act_pos = odrv0.axis0.pos_estimate
        except AttributeError:
            # Fallback for some firmware versions
            act_vel = odrv0.axis0.encoder.vel_estimate
            act_pos = odrv0.axis0.encoder.pos_estimate
            
        # Read phase current (how hard the motor is physically PUSHING)
        try:
            iq_set = odrv0.axis0.motor.foc.Iq_setpoint
            iq_meas = odrv0.axis0.motor.foc.Iq_measured
        except AttributeError:
            # Fallback
            iq_set = odrv0.axis0.motor.current_control.Iq_setpoint
            iq_meas = odrv0.axis0.motor.current_control.Iq_measured
            
        print(f"{t:>7.2f}s | {cmd_vel:>15.3f} | {act_vel:>15.3f} | {act_pos:>10.3f} | {iq_set:>10.3f} | {iq_meas:>10.3f}")
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("\nDiagnostic complete.")
