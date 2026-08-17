import odrive
import sys

print("Looking for ODESC...")
try:
    odesc = odrive.find_any(serial_number='336635923034', timeout=10)
except Exception as e:
    print(f"Error finding ODESC: {e}")
    sys.exit(1)

if odesc:
    print("Found ODESC! Setting config...")
    odesc.axis0.encoder.config.use_index = False
    odesc.axis0.encoder.config.pre_calibrated = False
    odesc.axis0.config.startup_encoder_index_search = False
    odesc.axis0.config.startup_encoder_offset_calibration = True
    odesc.axis0.motor.config.calibration_current = 25.0
    print("Saving configuration...")
    try:
        odesc.save_configuration()
    except:
        pass
    print("ODESC FIXED!")
else:
    print("ODESC not found.")
