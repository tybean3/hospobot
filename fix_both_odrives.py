import odrive
import time

print("Scanning for ODrives on USB...")
s1 = odrive.find_any(serial_number='395635433231', timeout=5)
if s1:
    print("Fixing S1 gains...")
    s1.axis0.controller.config.vel_gain = 0.8
    s1.axis0.controller.config.vel_integrator_gain = 5.0
    try:
        s1.save_configuration()
    except Exception as e:
        print("Saved S1 (rebooting).")
else:
    print("S1 not found. Ensure USB is plugged in.")

time.sleep(2)

odesc = odrive.find_any(serial_number='336635923034', timeout=5)
if odesc:
    print("Fixing ODESC Z-index issue...")
    # The floating/noisy Z-index pin causes the commutation angle to be randomized on boot.
    # We disable index search and force an offset calibration on boot.
    odesc.axis0.encoder.config.use_index = False
    odesc.axis0.encoder.config.pre_calibrated = False
    odesc.axis0.config.startup_encoder_index_search = False
    odesc.axis0.config.startup_encoder_offset_calibration = True
    # Increase calibration current slightly to ensure it can move the heavy wheel on the floor
    odesc.axis0.motor.config.calibration_current = 25.0
    try:
        odesc.save_configuration()
    except Exception as e:
        print("Saved ODESC (rebooting).")
else:
    print("ODESC not found. Ensure USB is plugged in.")

print("\nDone! Please reboot the robot (power off and on) so the changes take effect.")
print("When you power it on, the right wheel will automatically twitch to calibrate its offset.")
