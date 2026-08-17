import sys
try:
    import odrive
except ImportError:
    print("Error: 'odrive' python package not found.")
    sys.exit(1)

print("Searching for ODrive S1 via USB (Please ensure it is plugged in!)...")
# Find specifically the S1
odrv0 = odrive.find_any(serial_number='56514078453812', timeout=10)

if not odrv0:
    print("Could not find ODrive S1. Make sure the USB is plugged in.")
    sys.exit(1)
    
print(f"Connected to ODrive S1 (Serial: {odrv0.serial_number})")

print("\n--- Current Configuration ---")
try:
    print(f"Control Mode: {odrv0.axis0.controller.config.control_mode}")
    print(f"Input Mode: {odrv0.axis0.controller.config.input_mode}")
    print(f"Current Soft Max: {odrv0.axis0.config.motor.current_soft_max} A")
    print(f"Current Hard Max: {odrv0.axis0.config.motor.current_hard_max} A")
    if hasattr(odrv0.axis0.config, 'torque_soft_max'):
        print(f"Torque Soft Max: {odrv0.axis0.config.torque_soft_max} Nm")
    else:
        print(f"Torque Limit: {odrv0.axis0.controller.config.torque_limit} Nm")
except Exception as e:
    print(f"Error reading config: {e}")

print("\n--- Applying Fixes ---")
try:
    # 1. Ensure it's in Passthrough Velocity Control
    odrv0.axis0.controller.config.control_mode = 2 # VELOCITY_CONTROL
    odrv0.axis0.controller.config.input_mode = 1   # PASSTHROUGH
    
    # 2. Increase Current Limits to give it enough power to push the robot
    odrv0.axis0.config.motor.current_soft_max = 30.0
    odrv0.axis0.config.motor.current_hard_max = 40.0
    
    # 3. Increase Torque Limits
    if hasattr(odrv0.axis0.config, 'torque_soft_max'):
        odrv0.axis0.config.torque_soft_max = 2.0
    else:
        odrv0.axis0.controller.config.torque_limit = 2.0
        
    odrv0.save_configuration()
    print("SUCCESS: Limits increased and configuration saved to ODrive S1 memory!")
    
except Exception as e:
    print(f"Error applying fixes: {e}")
