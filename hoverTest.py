from dronekit import connect, VehicleMode, mavutil
import time

targetAlt = 4  # in meters
convertedAlt = targetAlt * 0.3048

startTime = time.time()
print("Start at 0 seconds")

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  # baud rate changed from 57600

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

print("Start hovering for 10 seconds")

vehicle.mode = VehicleMode("BRAKE")
vehicle.armed = False
while not vehicle.mode.name=='BRAKE' and vehicle.armed:
    print("BRAKING")
    time.sleep(1)

print(f"Hovering. Time: {time.time() - startTime}")
time.sleep(10)
print(f"Hovering done. Time: {time.time() - startTime}. Landing.")

vehicle.mode = VehicleMode("LAND")
vehicle.armed = False
while vehicle.armed:
    print("LANDING")
    time.sleep(1)

print(f"DONE. Total time: {time.time() - startTime} LETS GOOO")
