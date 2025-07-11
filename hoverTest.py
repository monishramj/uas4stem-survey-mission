from dronekit import connect, VehicleMode, mavutil
import time
import csv
import os

csvfile = open("hoverTest_DataContainingThePictures.csv", "a", newline='')
csv_writer = csv.writer(csvfile)

if os.stat("hoverTest_DataContainingThePictures.csv").st_size == 0:
    csv_writer.writerow(["Timestamp", "Event", "Value"])

targetAlt = 5 
convertedAlt = targetAlt * 0.3048
startTime = time.time()
print("Start at 0 seconds")

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  

while not vehicle.is_armable:
    print("Waiting to initialise...")
    time.sleep(1)

connectAndInitTime = time.time() - startTime
print(f"Initialised. Time to connect and initialise: {connectAndInitTime}")

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
    print("Arming")
    time.sleep(1)

armTime = time.time() - connectAndInitTime
print(f"Armed. Current Time: {time.time() - startTime}, time to arm: {armTime}")
print("Takeoff")

vehicle.simple_takeoff(convertedAlt)

altReachTime = time.time()
while True:
    print(f"Altitude:{vehicle.location.global_relative_frame.alt}")
    if vehicle.location.global_relative_frame.alt >= convertedAlt * 0.95:
        altReachTime = time.time() - armTime
        print(f"Reached target altitude. Current time: {time.time() - startTime}, time to reach altitude: {altReachTime}")
        break
    time.sleep(1)

print("Start hovering for 5 seconds.")

vehicle.mode = VehicleMode("BRAKE")
vehicle.armed = False
while not vehicle.mode.name == 'BRAKE' and vehicle.armed:
    print("BRAKING")
    time.sleep(1)

breakTime = time.time() - altReachTime
print(f"Hovering. Current time: {time.time() - startTime}, time to break: {breakTime}")
time.sleep(5)
hoverTime = time.time() - breakTime
print(f"Hovering done. Current time: {time.time() - startTime}. Landing, time to hover: {hoverTime}")

vehicle.mode = VehicleMode("LAND")
vehicle.armed = False
while vehicle.armed:
    print("LANDING")
    time.sleep(1)

landingTime = time.time() - hoverTime
print(f"DONE. Total time: {time.time() - startTime} LETS GOOO.\nTime to connect and initialise: {connectAndInitTime}\nTime to arm: {armTime}\nTime to takeoff and reach altitude: {altReachTime}\nTime to break/get into hover: {breakTime}\nTime to hover compared to 5 second coded time: {hoverTime}\nTime to land: {landingTime}")

timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
csv_writer.writerow([timestamp, "Time to Connect and Initialise (s)", f"{connectAndInitTime:.2f}"])
csv_writer.writerow([timestamp, "Time to Arm (s)", f"{armTime:.2f}"])
csv_writer.writerow([timestamp, "Time to Takeoff and Reach Altitude (s)", f"{altReachTime:.2f}"])
csv_writer.writerow([timestamp, "Time to Break/Get into Hover (s)", f"{breakTime:.2f}"])
csv_writer.writerow([timestamp, "Time to Hover (s)", f"{hoverTime:.2f}"])
csv_writer.writerow([timestamp, "Time to Land (s)", f"{landingTime:.2f}"])

csvfile.close()
