from dronekit import connect, VehicleMode
import time, math, traceback
from enum import Enum
from datetime import datetime
from picamera2 import Picamera2 
import numpy as np
import cv2 as cv

# * TODO: competition algorithm that uses homography-esque algorithm to determine coordinates from scanning
# * no movmement apart from AUTO mission, purely computer vision

class Targets(Enum):
    TOWER = 0
    TREE = 1
    RUBBLE = 2
    LUMBER = 3
    PIPES = 4
    POWER_LINES = 5
    INTERSECTION = 6
    TRASH = 7
    PERSONNEL = 8
    BRIDGE = 9
    HARDWARE = 10
    EQUIPMENT = 11

# -- HELPER FUNCTIONS
def detectSIFT(frame, contour):
    kp, desc = sift.detectAndCompute(frame, None) # https://amroamroamro.github.io/mexopencv/matlab/cv.SIFT.detectAndCompute.html
    best = 0
    bestID = -1
    bestMatches = False
    for i in range(12):
        if i == 6:
            ratio = 0.6
        else:
            ratio = 0.5
        matches = bf.knnMatch(siftDesc[i], desc, k=2) # https://docs.opencv.org/3.4/dc/dc3/tutorial_py_matcher.html
        validMatches = []
        if len(matches) != 0 and len(matches[0]) == 2:
            avr = 0
            for m, n in matches:
                if min(m.distance / n.distance, n.distance / m.distance) < ratio:
                #if min(m.distance / n.distance, n.distance / m.distance) < ratio and cv.pointPolygonTest(contour, kp[n.queryIdx].pt, True) <= 0:
                    validMatches.append(m)
                    avr += m.distance / n.distance
            if len(validMatches) != 0:
                avr /= len(validMatches)
            else:
                continue
            avr = len(validMatches) / len(matches)
        if avr > best:
            best = avr
            bestID = i
            bestMatches = validMatches
    if best != 0:
        return [bestID, bestMatches]
    else:
        return bestMatches
def findPOI(frame):
    ret, thresh = cv.threshold(frame, 200, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    validContours = []
    for i in range(len(contours)):
        contour = contours[i]

        #check area
        if cv.contourArea(contour) < 150:
            continue

        #check solidity
        hull = cv.convexHull(contour)
        if cv.contourArea(hull) == 0:
            continue
        solidity = abs(cv.contourArea(contour) / cv.contourArea(hull))
        if abs(solidity - 1) > 0.1:
            continue

        #check approx
        epsilon = 0.02 * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        if len(approx) != 4:
            continue

        if hierarchy[0][i][3] != -1:
            continue

        validContours.append(contour)
    validContours.sort(key=lambda contour: -cv.contourArea(contour))
    return validContours
def find_side_length(contour):
    epsilon = .02 * cv.arcLength(contour, True)
    approx = cv.approxPolyDP(contour, epsilon, True)

    pts = approx.reshape(4,2)
    side1 = np.linalg.norm(pts[0] - pts[1])
    side2 = np.linalg.norm(pts[1] - pts[2])
    side3 = np.linalg.norm(pts[2] - pts[3])
    side4 = np.linalg.norm(pts[3] - pts[0])

    avg_side = (side1 + side2 + side3 + side4) / 4
    return avg_side
def calculate_target_gps(drone, dx_dist, dy_dist): # function takes in dx dy feet and gives new GPS coordinates
    screen_angle = math.degrees(math.atan2(dy, dx))
    #global_angle_deg = -screen_angle + 90 + drone.heading
    global_angle_rad = math.radians(-1 * screen_angle + drone.heading)

    total_dist_ft = math.sqrt(dx_dist**2 + dy_dist**2)
    
    north_ft = total_dist_ft * math.cos(global_angle_rad)
    east_ft = total_dist_ft * math.sin(global_angle_rad)

    north_m = north_ft * 0.3048
    east_m = east_ft * 0.3048

    R = 6378137.0

    drone_lat = drone.location.global_frame.lat
    drone_lon = drone.location.global_frame.lon
    
    d_lat = north_m / R
    d_lat_deg = d_lat * 180 / math.pi
    
    d_lon = east_m / (R * math.cos(math.pi * drone_lat / 180))
    d_lon_deg = d_lon * 180 / math.pi
    
    target_lat = drone_lat + d_lat_deg
    target_lon = drone_lon + d_lon_deg
    
    return target_lat, target_lon
def arm_takeoff(altitude, vehicle): # in feet
    convertedAlt = altitude * 0.3048 # in meters

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
def set_mode(vehicle, mode):
    mode = mode.upper()
    print(f"Start set to {mode}")
    vehicle.mode = VehicleMode(mode)
    while not vehicle.mode.name==mode:
        print(f"Setting {mode}")
        time.sleep(1)
    print(f"{mode} set worked: drone mode is {vehicle.mode.name}!") #checking to make sure it is set
    return

#! 1.----- CONNECT THE DRONE -----
current_date_and_time = datetime.now()
print("The current date and time of flight is", current_date_and_time)
print("Trying to connect...")
drone = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200) 
print("Connected BOOYAH!") 

#! 2.----- LAUNCH ------
arm_takeoff(30, drone)

#! 3.----- IMAGING SETUP -----
print("Begin image setup")
sift = cv.xfeatures2d.SIFT_create()
bf = cv.BFMatcher()

templates = []
templateSizes = []
siftKP = []
siftDesc = []
for i in range(12): 
    templates.append(cv.imread('images/' + str(i+1) + '.png', cv.IMREAD_GRAYSCALE))
    templates[i] = cv.resize(templates[i], None, fx=0.5, fy=0.5, interpolation=cv.INTER_AREA)
    templateSizes.append(templates[i].shape[::-1])
    kp, desc = sift.detectAndCompute(templates[i], None)
    siftKP.append(kp)
    siftDesc.append(desc)

print("Begin PI setup")

WIDTH, HEIGHT = 1280, 720
centerX = WIDTH/2
centerY = HEIGHT/2
vid = Picamera2()
vid.video_configuration.controls.FrameRate = 20.0
vid.configure(vid.create_video_configuration({"size": (WIDTH,HEIGHT), "format": "BGR888"}))
vid.start()
out = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*'MJPG'), 20.0, (WIDTH,HEIGHT))
print("Video start")

vid.set_controls({
    "ExposureTime": 2000,
    "AnalogueGain": 1.1,
    "Saturation": 0.0,
    "Contrast": 1.1,
})

print("Control effects set")
dx = 0
dy = 0
dx_dist = 0
dy_dist = 0
lat = 0
lon = 0
target_id = -1
target_name = ''
font = cv.FONT_HERSHEY_SIMPLEX

set_mode(drone, "AUTO")

time.sleep(5)

while True:
    # If RTL engaged, break
    if drone.mode.name == 'RTL':
        print("RTL detected, mission END")
        break
    frame = vid.capture_array()

    # filter green
    filtered = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    lowerGreen = np.array([40, 255, 255])
    upperGreen = np.array([80, 0, 0])
    maskA = cv.inRange(filtered, np.array([0, 0, 0]), lowerGreen)
    maskB = cv.inRange(filtered, upperGreen, np.array([255, 255, 255]))
    mask = cv.bitwise_or(maskA, maskB)
    filtered = cv.bitwise_and(filtered, filtered, mask=mask)
    filtered = cv.cvtColor(filtered, cv.COLOR_HSV2BGR)
    filtered = cv.cvtColor(filtered, cv.COLOR_BGR2GRAY)

    cv.putText(frame, f'GPS: ({drone.location.global_frame.lat}, {drone.location.global_frame.lon})', (10, 30), font, 1, (0, 255, 255), 2, cv.LINE_4)

    #! EACH FRAME, CHECKS FOR POIS
    contours = findPOI(filtered)
    frame = cv.drawContours(frame, contours, -1, (0, 255, 0), 2)

    if len(contours) != 0:        
        print("found something")             
        M = cv.moments(contours[0])
        targetX = int(M["m10"] / M["m00"])
        targetY = int(M["m01"] / M["m00"])
        cv.circle(frame, (targetX, targetY), 5, (0, 255, 0), -1)
        dx = centerX - targetX
        dy = centerY - targetY
        dist_from_center = math.hypot(dx, dy)

        avg_side = find_side_length(contours[0])

        TARGET_SIDE_LENGTH = 4 
        dx_dist = (dx/avg_side) * TARGET_SIDE_LENGTH
        dy_dist = (dy/avg_side) * TARGET_SIDE_LENGTH
        approx_dist = (dist_from_center/avg_side) * TARGET_SIDE_LENGTH
        lat, lon = calculate_target_gps(drone, dx_dist, dy_dist)
    
    kp, desc = sift.detectAndCompute(filtered, None)
    if len(contours) != 0:
        detect = detectSIFT(filtered, contours[0])
    else:
        detect = False
        target_id = -1
        target_name = ''
    if detect != False:
        frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        target_id = detect[0]
        matches = detect[1]
        target_name = Targets(target_id).name
        print(f"Target_name: {target_name}")

    cv.putText(frame, f'Target {target_id}: {target_name}', (10, 70), font, 1, (0, 255, 255), 2, cv.LINE_4)
    cv.putText(frame, f'Calculated GPS: ({lat}, {lon})', (10, 110), font, 1, (0, 255, 255), 2, cv.LINE_4)
    cv.putText(frame, f'px: ({dx}, {dy}). ft: ({dx_dist}, {dy_dist}), off by {approx_dist}', (10, 110), font, 1, (0, 255, 255), 2, cv.LINE_4)
    out.write(frame)
    if cv.waitKey(1) == ord('q'):
            break

out.release()
print("Preparing to return and land")
