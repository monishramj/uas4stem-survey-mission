from dronekit import connect, VehicleMode, LocationGlobal
import time, math
from mavlink.pymavlink import mavutil

def localCoordinate(vehicle, north, east, down): # function converts feet to new LocationGlobal 
    start_pos = vehicle.location.global_frame
    start_lat = start_pos.lat
    start_lon = start_pos.lon
    start_alt = start_pos.alt

    north_m = north * .3048
    east_m = east * .3048
    down_m = down * .3048

    R = 6378137.0

    d_lat = north_m / R
    d_lon = east_m / (R * math.cos(math.pi * start_lat / 180))

    final_lat = start_lat + (d_lat * 180 / math.pi)
    final_lon = start_lon + (d_lon * 180 / math.pi)
    final_alt = start_alt - down_m

    return LocationGlobal(final_lat, final_lon, final_alt)
# original movement test written by Nidhish

targetAlt = 5  # in feet

convertedAlt = targetAlt * 0.3048 # in meters

startTime = time.time()
print("Start at 0 seconds")

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  # baud rate changed from 57600

vehicle.location.local_frame

while not vehicle.is_armable:
    print("Waiting to initialise...")
    time.sleep(1)

print(f"Time: {time.time() - startTime}")

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
    print("Arming")
    time.sleep(1)

print(f"Armed. Time: {time.time() - startTime}")
print("Takeoff")

vehicle.simple_takeoff(convertedAlt)

while True:
    print(f"Altitude:{vehicle.location.global_relative_frame.alt}")
    if vehicle.location.global_relative_frame.alt>= convertedAlt * 0.95:
        print(f"Reached target altitude. Time: {time.time() - startTime}")
        break
    time.sleep(1)

print(f"Starting to move, Global Frame: {vehicle.location.global_frame}")
move = localCoordinate(vehicle, 5, 5, 0)

print(f"New coordinates: {move}")
vehicle.simple_goto(move)
time.sleep(5)
print("Moving finished")

vehicle.mode = VehicleMode("LAND")
vehicle.armed = False
while vehicle.armed:
    print("LANDING")
    time.sleep(1)

print(f"DONE. Total time: {time.time() - startTime} LETS GOOO")