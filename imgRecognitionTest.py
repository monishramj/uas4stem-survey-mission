# * TODO: create algorithm on coming to altitude and moving drone to center image
# 1. connect
# 2. launch and save initial gps
# 3. gain input on any 'white' image on ground
# 4. TEST algorithm to determine distance of movement depending on distance from center of camera on screen
# -- something haivng to do with calculating and translating vector?
# 5. move drone to that specified local coordinate
# 6. run TEST algorithm to double check center alignment of image
# 7. repeat TEST movement until centered
# 8. get gps coordinates
# 9. run opencv to determine image type
# 10. go back to initial gps and land

from dronekit import connect, VehicleMode, mavutil, LocationLocal
import time
# original movement test written by Nidhish

targetAlt = 5  # in feet
convertedAlt = targetAlt * 0.3048 # in meters

startTime = time.time()
print("Start at 0 seconds")

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  # baud rate changed from 57600

while not vehicle.is_armable:
    print("Waiting to initialise...")
    time.sleep(1)
