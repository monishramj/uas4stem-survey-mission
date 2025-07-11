from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time, video, math
import numpy as np
import cv2 as cv

sift = cv.xfeatures2d.SIFT_create()
bf = cv.BFMatcher()
templates = []
templateSizes = []
siftKP = []
siftDesc = []

# * TODO: create algorithm on coming to altitude and moving drone to center image

# 2 m/s ()

def create_local_coordinate(vehicle, north, east, down): # function converts feet to new LocationGlobal 
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

#! 1. connect x
print("trying to connect")
vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  

#! 2. launch and save initial gps x
arm_takeoff(30, vehicle)
initial_pos = vehicle.location.global_frame

start_hover(vehicle) #hover while it scans

#! 3. gain input on any 'white' image on ground
CENTER_THRESHOLD = 20 # pixels to center 

vid = cv.VideoCapture(0)

if not vid.isOpened():
    print("Failed to open camera.")
    land(vehicle)

centerX = vid.get(cv.CAP_PROP_FRAME_WIDTH)/2
centerY = vid.get(cv.CAP_PROP_FRAME_HEIGHT)/2

out = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*'MJPG'), 60.0, (640, 480))

dx = 0 #distance from center of img to center of screen
dy = 0 #distance from center of img to center of screen

while True:
    ret, frame = vid.read()
    
    if not ret:
        print("Failed to recieve frame.")

    #filter out green
    filtered = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lowerGreen = np.array([40, 255, 255])
    upperGreen = np.array([80, 0, 0])
    maskA = cv.inRange(filtered, np.array([0, 0, 0]), lowerGreen)
    maskB = cv.inRange(filtered, upperGreen, np.array([255, 255, 255]))
    mask = cv.bitwise_or(maskA, maskB)
    filtered = cv.bitwise_and(filtered, filtered, mask=mask)
    filtered = cv.cvtColor(filtered, cv.COLOR_HSV2BGR)
    filtered = cv.cvtColor(filtered, cv.COLOR_BGR2GRAY)

    contours = video.findPOI(filtered)
    frame = cv.drawContours(frame, contours, -1, (0, 255, 0), 2)

    if len(contours) != 0:
        M = cv.moments(contours[0])
        targetX = int(M["m10"] / M["m00"])
        targetY = int(M["m01"] / M["m00"])
        cv.circle(frame, (targetX, targetY), 5, (0, 255, 0), -1)
        dx = centerX - targetX
        dy = centerY - targetY
        print(f"Target offset: dx={dx:.2f}, dy={dy:.2f}")

        if abs(dx) > CENTER_THRESHOLD and abs(dy) > CENTER_THRESHOLD:
            move_north = dy / 20
            move_east = -dx / 20
            new_loc = create_local_coordinate()
        else: 
            print("Target centered.")
            print("")
            break


    kp, desc = sift.detectAndCompute(filtered, None)
    if len(contours) != 0:
        detect = video.detectSIFT(filtered, contours[0])
    else:
        detect = False
    if detect:
        frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    out.write(frame)
    cv.imshow('stream', frame)
    if cv.waitKey(1) == ord('q'):
        break

out.release()
vid.release()
cv.destroyAllWindows()

vehicle.simple_goto(initial_pos)
land(vehicle)

