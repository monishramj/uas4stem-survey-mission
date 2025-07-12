from dronekit import connect, VehicleMode, Vehicle, LocationGlobal
import time, math
from pymavlink import mavutil

def send_ned_velocity(vehicle, north, east, altitude, duration):
   msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system and component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only velocities enabled)
        north, east, altitude, # x, y, z velocity in m/s
        # altitude is positive down
        0, 0, 0, # x, y, z acceleration in m/s/s (not used)
        0, 0, 0, # x, y, z position in m (not used)
        0, 0)    # yaw and yaw_rate (not used)
   
   for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def arm_takeoff(altitude, vehicle): # in feet
    convertedAlt = altitude * 0.3048 # in meters

    #! 2. launch and save initial gps x
    while not vehicle.is_armable:
        print("Waiting to initialise...")
        time.sleep(1)

    print(f"Initialised.")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
        print("Arming")
        time.sleep(1)

    print(f"Armed. Takeoff!")

    vehicle.simple_takeoff(convertedAlt)

    altReachTime = time.time()
    while True:
        print(f"Altitude:{vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt>= convertedAlt * 0.95:
            print(f"Reached target altitude.")
            break
        time.sleep(1)
    
    return

def start_hover(vehicle):
    vehicle.mode = VehicleMode("BRAKE")
    vehicle.armed = False
    while not vehicle.mode.name=='BRAKE' and vehicle.armed:
        print("BRAKING")
        time.sleep(1)
    return

def land(vehicle):
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed = False
    while vehicle.armed:
        print("LANDING")
        time.sleep(1)

targetAlt = 20  # in feet
convertedAlt = targetAlt * 0.3048 # in meters

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  # baud rate changed from 57600

arm_takeoff(convertedAlt, vehicle)

print(f"Starting to move Global Frame: {vehicle.location.global_frame}")
send_ned_velocity(vehicle, 1, 0, 0, 5)
print(f"Moving finished. Global Frame: {vehicle.location.global_frame}")

land(vehicle)

print(f"DONE. LETS GOOO")