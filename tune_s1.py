import sys
try:
    import odrive
except ImportError:
    print("Error: 'odrive' python package not found.")
    sys.exit(1)

print("Searching for ODrive S1 via USB...")
odrv0 = odrive.find_any(serial_number='395635433231', timeout=10)

if not odrv0:
    print("Could not find ODrive S1. Make sure it is plugged in and no other scripts are using it.")
    sys.exit(1)

print(f"Connected to ODrive S1 (Serial: {odrv0.serial_number})")

try:
    # Current values
    print("\n--- Current Tuning ---")
    print(f"Vel Gain: {odrv0.axis0.controller.config.vel_gain:.4f}")
    print(f"Vel Integrator Gain: {odrv0.axis0.controller.config.vel_integrator_gain:.4f}")
    
    # Increase the Proportional gain significantly more! 
    # If 0.5 was too weak, 3.0 should make it very stiff and responsive.
    new_vel_gain = 3.0 
    
    # Increase the Integrator to match the new proportional gain
    new_vel_integrator_gain = 15.0
    
    print("\n--- Applying New Tuning ---")
    odrv0.axis0.controller.config.vel_gain = new_vel_gain
    odrv0.axis0.controller.config.vel_integrator_gain = new_vel_integrator_gain
    
    odrv0.save_configuration()
    print(f"SUCCESS: vel_gain set to {new_vel_gain}, vel_integrator_gain set to {new_vel_integrator_gain}")
    print("Test it now! It should be much harder to stop by hand.")
    
except Exception as e:
    print(f"Error applying tuning: {e}")
