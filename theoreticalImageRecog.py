from dronekit import connect, VehicleMode, LocationGlobal
from pymavlink import mavutil
import time, math, video
import numpy as np
import cv2 as cv

sift = cv.xfeatures2d.SIFT_create()
bf = cv.BFMatcher()
templates = []
templateSizes = []
siftKP = []
siftDesc = []

CENTER_THRESHOLD = 20

def create_local_coordinate(vehicle, north, east, down):
    start_pos = vehicle.location.global_frame
    north_m = north * 0.3048
    east_m = east * 0.3048
    down_m = down * 0.3048
    R = 6378137.0

    d_lat = north_m / R
    d_lon = east_m / (R * math.cos(math.pi * start_pos.lat / 180))

    final_lat = start_pos.lat + (d_lat * 180 / math.pi)
    final_lon = start_pos.lon + (d_lon * 180 / math.pi)
    final_alt = start_pos.alt - down_m

    return LocationGlobal(final_lat, final_lon, final_alt)

print("Connecting...")
vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)

targetAlt_ft = 30
targetAlt_m = targetAlt_ft * 0.3048

while not vehicle.is_armable:
    print("Waiting for vehicle to initialise...")
    time.sleep(1)

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print("Arming...")
    time.sleep(1)

print("Taking off...")
vehicle.simple_takeoff(targetAlt_m)

while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f"Altitude: {alt:.2f} m")
    if alt >= targetAlt_m * 0.95:
        print("Reached target altitude.")
        break
    time.sleep(1)

vid = cv.VideoCapture(0)
if not vid.isOpened():
    print("Failed to open camera.")
    vehicle.mode = VehicleMode("LAND")
    exit()

centerX = vid.get(cv.CAP_PROP_FRAME_WIDTH) / 2
centerY = vid.get(cv.CAP_PROP_FRAME_HEIGHT) / 2
out = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*'MJPG'), 60.0, (640, 480))

while True:
    ret, frame = vid.read()
    if not ret:
        print("Failed to read frame.")
        continue

    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    maskA = cv.inRange(hsv, np.array([0, 0, 0]), np.array([40, 255, 255]))
    maskB = cv.inRange(hsv, np.array([80, 0, 0]), np.array([255, 255, 255]))
    mask = cv.bitwise_or(maskA, maskB)
    filtered = cv.bitwise_and(hsv, hsv, mask=mask)
    gray = cv.cvtColor(filtered, cv.COLOR_HSV2BGR)
    gray = cv.cvtColor(gray, cv.COLOR_BGR2GRAY)

    contours = video.findPOI(gray)
    frame = cv.drawContours(frame, contours, -1, (0, 255, 0), 2)

    if contours:
        M = cv.moments(contours[0])
        if M["m00"] != 0:
            targetX = int(M["m10"] / M["m00"])
            targetY = int(M["m01"] / M["m00"])
            dx = centerX - targetX
            dy = centerY - targetY
            print(f"Target offset: dx={dx:.2f}, dy={dy:.2f}")
            cv.circle(frame, (targetX, targetY), 5, (0, 255, 0), -1)

            if abs(dx) > CENTER_THRESHOLD or abs(dy) > CENTER_THRESHOLD:
                move_north = -dy / 20 
                move_east = -dx / 20
                new_loc = create_local_coordinate(vehicle, move_north, move_east, 0)
                print(f"Moving to: N={move_north:.2f}ft, E={move_east:.2f}ft")
                vehicle.simple_goto(new_loc, groundspeed=1.5)
                time.sleep(5)

    kp, desc = sift.detectAndCompute(gray, None)
    detect = video.detectSIFT(gray, contours[0]) if contours else False
    if detect:
        frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    out.write(frame)
    cv.imshow('stream', frame)
    if cv.waitKey(1) == ord('q'):
        break

print("Landing...")
vehicle.mode = VehicleMode("LAND")
vehicle.armed = False
while vehicle.armed:
    print("Landing...")
    time.sleep(1)

out.release()
vid.release()
cv.destroyAllWindows()

