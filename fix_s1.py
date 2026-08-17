import odrive
import sys

print("Looking for S1...")
try:
    s1 = odrive.find_any(serial_number='395635433231', timeout=10)
except Exception as e:
    print(f"Error finding S1: {e}")
    sys.exit(1)

if s1:
    print("Found S1! Setting gains...")
    s1.axis0.controller.config.vel_gain = 0.8
    s1.axis0.controller.config.vel_integrator_gain = 5.0
    print("Saving configuration...")
    try:
        s1.save_configuration()
    except:
        pass
    print("S1 FIXED!")
else:
    print("S1 not found.")
