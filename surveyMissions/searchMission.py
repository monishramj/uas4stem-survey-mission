from dronekit import connect, VehicleMode, LocationGlobal, mavutil
import time, math, traceback
from enum import Enum
from datetime import datetime
from picamera2 import Picamera2 
import numpy as np
import cv2 as cv
# * TODO: competition algorithm that uses imageRecognition algorithm within a multi-POI auto-mission setup
# * should move to position drone onto img

# -- ENUM FOR TARGET IDS
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
def calculate_target_gps(drone, dx_dist, dy_dist): # function takes in dx dy feet and gives new GPS coordinates
    screen_angle = math.degrees(math.atan2(dy_dist, dx_dist))
    global_angle_deg = -screen_angle + 90 + drone.heading
    global_angle_rad = math.radians(global_angle_deg)

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
def land(vehicle):
   vehicle.mode = VehicleMode("LAND")
   vehicle.armed = False
   while vehicle.armed:
        print("LANDING")
        time.sleep(1)

#! 1.----- CONNECT THE DRONE -----
current_date_and_time = datetime.now()
print("The current date and time of flight is", current_date_and_time)
print("Trying to connect...")
drone = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200) 
print("Connected BOOYAH!") 

#! 2.----- LAUNCH ------
arm_takeoff(30, drone)

set_mode(drone, "AUTO")

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

#! 4.----- PI SETUP -------
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

#! 5.------ SEARCH SETUP ------
found_targets = []  # store detected target IDs
dx, dy = 0, 0
font = cv.FONT_HERSHEY_SIMPLEX

CENTER_THRESHOLD = 100  # pixel amt to be centered
MOVE_TIME = 2 # sec, how long drone has to move
STEP_LENGTH = 2  # ft, how far mvm is when detected
STEP_SPEED = 1.5 # m/s, GROUND speed for mvm to coordinate
TARGET_COOLDOWN = 5 # sec, how long drone waits before checking for POIs again
CENTERING_TIMEOUT = 30 #sec, how long it takes to try and center
print("Search initated")

# COOLDOWNS/TIMERS
moving = None
scan_cd = None # target cooldown to not keep scanning same thing
center_cd = None # checks center cooldown
current_target = None

scan_cd = time.time() + TARGET_COOLDOWN
#! 6.------ SEARCH ------
try:
    while True:
        #! EACH FRAME, WRITES CAPTURE AND GREEN FILTER
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

        #! EACH FRAME, CHECKS FOR POIS
        contours = findPOI(filtered)
        frame = cv.drawContours(frame, contours, -1, (0, 255, 0), 2)

        # If RTL engaged, break
        if drone.mode.name == 'RTL':
            print("RTL detected, mission END")
            break
            
        #! ONLY WHEN MOVING TO POI
        if drone.mode.name == 'GUIDED': 
            if moving is not None: 
                if time.time() >= moving:
                    print("Finishing positioning, back to AUTO!")
                    set_mode(drone, "AUTO")
                    moving = None

        #! AUTO SURVEY WHILE WAYPOINT TAKES CONTROL
        if drone.mode.name == 'AUTO':
            if len(contours) != 0 and moving is None:
                #! SCAN COOLDOWN TO NOT SCAN SAME THING
                if scan_cd is not None:
                    if time.time() >= scan_cd:
                        scan_cd = None
                    else:
                        continue
                set_mode(drone, 'BRAKE')
                print("Possible target")
                kp, desc = sift.detectAndCompute(filtered, None)
                detect = detectSIFT(filtered, contours[0])
                if detect != False:
                    target_id = detect[0]
                    matches = detect[1]
                    frame = cv.drawMatches(templates[target_id], siftKP[target_id], frame, kp, matches, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                    
                    target_name = Targets(target_id).name
                    if target_id in found_targets:
                        print(f"Already found target {target_id}: {target_name}, ignore ts")
                        set_mode(drone, 'AUTO')
                        scan_cd = time.time() + TARGET_COOLDOWN
                        continue
                    print(f"New target {target_id}: {target_name} found")

                    if current_target != target_id:
                        current_target = target_id
                        center_cd = time.time() + CENTERING_TIMEOUT
                        print(f"Checking centering for target {target_id}")

                    if center_cd != None and time.time() >= center_cd:
                        print(f"Centering (current: {current_target} for id: {target_id} TIMEOUT")
                        print(f"GPS coordinate: lat:{drone.location.global_frame.lat}, lon:{drone.location.global_frame.lon}")
                        scan_cd = time.time() + TARGET_COOLDOWN
                        current_target = None
                        set_mode(drone, 'AUTO')
                        continue

                    # experimental get approx. side length
                    contour = contours[0]
                    epsilon = .02 * cv.arcLength(contour, True)
                    approx = cv.approxPolyDP(contour, epsilon, True)

                    pts = approx.reshape(4,2)
                    side1 = np.linalg.norm(pts[0] - pts[1])
                    side2 = np.linalg.norm(pts[1] - pts[2])
                    side3 = np.linalg.norm(pts[2] - pts[3])
                    side4 = np.linalg.norm(pts[3] - pts[0])

                    avg_side = (side1 + side2 + side3 + side4) / 4
                    # end experimental

                    # center offsets, IMPORTANT DONT COMMENT OUT
                    M = cv.moments(contours[0])
                    targetX = int(M["m10"] / M["m00"])
                    targetY = int(M["m01"] / M["m00"])
                    cv.circle(frame, (targetX, targetY), 5, (0, 255, 0), -1)
                    dx = -(centerX - targetX)
                    dy = centerY - targetY
                    dist_from_center = math.hypot(dx, dy)

                    # more experimental
                    # we're guesstimating by calculating dx, dy pixels and converting them to GPS coordinates
                    TARGET_SIDE_LENGTH = 4 # in ft
                    print(f"Target offset: dx={dx:.2f}, dy={dy:.2f}, dist={dist_from_center:.2f}")
                    dx_dist = (dx/avg_side) * TARGET_SIDE_LENGTH
                    dy_dist = (dy/avg_side) * TARGET_SIDE_LENGTH
                    approx_dist = (dist_from_center/avg_side) * TARGET_SIDE_LENGTH
                    cv.putText(frame, f'dx, dy: {int(dx)}, {int(dy)}, real: {dx_dist}ft, {dy_dist}ft', (10, 70), font, 1, (0, 255, 255), 2, cv.LINE_4)
                    approx_lat, approx_lon = calculate_target_gps(drone, dx_dist, dy_dist)
                    cv.putText(frame, f'~distToTarget: {int((dist_from_center/avg_side) * 4)}ft; ({approx_lat}, {approx_lon})', (10, 110), font, 1, (0, 255, 255), 2, cv.LINE_4)
                    # end experimental

                    if dist_from_center > CENTER_THRESHOLD:
                        print("Try move over target")
                        set_mode(drone, "GUIDED") #set guided to move

                        screen_angle = math.degrees(math.atan2(dy, dx))-90
                        global_angle = math.radians(-1 * screen_angle + drone.heading) # heading is calc. yaw, 0 is N, goes E i believe

                        move_north = STEP_LENGTH * math.cos(global_angle)
                        move_east = STEP_LENGTH * math.sin(global_angle)

                        new_loc = create_local_coordinate(drone, move_north, move_east, 0)
                        print(f"Moving to: N={move_north:.2f}ft, E={move_east:.2f}ft")

                        drone.simple_goto(new_loc, groundspeed=STEP_SPEED)
                        moving = time.time() + MOVE_TIME
                    else:
                        print(f"Target {target_id} centered.")
                        print(f"GPS coordinate: lat:{drone.location.global_frame.lat}, lon:{drone.location.global_frame.lon}")

                        found_targets.append(target_id)
                        scan_cd = time.time() + TARGET_COOLDOWN
                        current_target = None
                        set_mode(drone, "AUTO")

            # potentilaly deprecated code...
            # kp, desc = sift.detectAndCompute(filtered, None)
            # if len(contours) != 0:
            #     detect = detectSIFT(filtered, contours[0])
            # else:
            #     detect = False
            # if detect:
            #     frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        
        #! WRITE VALUES ON VIDEO
        # lat_txt = drone.location.global_frame.lat
        # lon_txt = drone.location.global_frame.lon
        cv.putText(frame, f'GPS: ({drone.location.global_frame.lat}, {drone.location.global_frame.lon})', (10, 30), font, 1, (0, 255, 255), 2, cv.LINE_4)
        
        
        out.write(frame) # write out frame w/ debug lines
    
    print("Mission complete")
except KeyboardInterrupt:
    print("Mission interrupted manually")
except Exception as e:
    print(f"ERROR. Mission interrupted by: {type(e).__name__} - {e}.")
    tb = traceback.format_exc()
    print(tb)  # Or write to a log file

out.release()
vid.release()
print("Preparing to return and land")
set_mode(drone, "RTL")
print("We done WOOOHOO (or not womp)")

