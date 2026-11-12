import sys
import pydobot
import time

# ---------------Connecting---------------
port = '/dev/ttyUSB0'
print("Connecting to Dobot...")
try:
    device = pydobot.Dobot(port=port, verbose=False)
    print('Dobot connected')
except Exception as e:
    # If connection fails, exit the script.
    print(f'Error. Dobot not connected: {e}')
    sys.exit(1) 

#--------------Parameters-------------------
home_x, home_y, home_z, home_r = 200.0, 0.0, 50.0, 0.0
fast_speed, fast_acc = 70, 70
norm_speed, norm_acc = 50, 50
slow_speed, slow_acc = 5, 40



try:
    #---------------Moving to starting position--------
    print("Moving to starting position")
    # FIX: Use a valid minimum speed and acceleration (e.g., 50, 50)
    device.speed(fast_speed, fast_acc)
    device.move_to(home_x, home_y, home_z, home_r, wait=True)

    # ------------------Exfoliation demo------------------------
    target_x = home_x
    target_y = home_y
    start_z = home_z
    target_r = home_r + 45.0  

    print(f"Starting position -> x:{target_x:.1f} y:{target_y:.1f} z:{start_z:.1f} r:{home_r:.1f}")

    print("Closing gripper...")
    device.grip(True)
    time.sleep(2)

    # Rotation
    device.speed(norm_speed, norm_acc)
    # Use target_r (exactly 45.0)
    device.move_to(target_x, target_y, start_z, target_r, wait=True)
    
    # Drop down 
    device.move_to(target_x, target_y, start_z - 35, target_r, wait=True) 
    
    device.speed(slow_speed, slow_acc)
    # X, Y, and R are completely identical to the previous line
    device.move_to(target_x, target_y, start_z - 50, target_r, wait=True) 
    
    # Pull back up
    device.move_to(target_x, target_y, start_z - 35, target_r, wait=True)
    
    device.speed(norm_speed, norm_acc)
    device.move_to(target_x, target_y, start_z, target_r, wait=True)

    # Back rotation
    device.move_to(target_x, target_y, start_z, home_r, wait=True)

    print("Opening gripper...")
    device.grip(False)
    time.sleep(2) 

    # ------------------Going to resting position------------------------
    device.speed(fast_speed, fast_acc)
    print("Moving arm to safe resting position...")
    device.move_to(home_x, home_y, home_z, home_r, wait=True)
    print("Arm parked safely.")

except Exception as e:
    print(f"Error: {e}")

finally:
    print("Closing connection...")
    device.close()
    print("Connection closed. It is now safe to turn off the physical power switch.")