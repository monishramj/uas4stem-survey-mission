from dronekit import VehicleMode, LocationGlobal, mavutil
import numpy as np
import cv2 as cv
import math, time
import constants
from picamera2 import Picamera2


#Preprocessing opencv
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

class Video:
    def __init__(self, type):
        self.type = type
        if type == "Pi":
            print("Using picamera")
            self.vid = Picamera2()
        else:
            self.vid = cv.VideoCapture(0)

    def start(self):
        if self.type == "Pi":
            self.vid.video_configuration.controls.FrameRate = constants.CAMERA_FPS
            self.vid.configure(self.vid.create_video_configuration({"size": (constants.WIDTH, constants.HEIGHT), "format": constants.CAMERA_FORMAT}))
            self.vid.set_controls({
                "ExposureTime": constants.EXPOSURE,
                "AnalogueGain": constants.GAIN,
                "Saturation": constants.SATURATION,
                "Contrast": constants.CONTRAST,
            })
            self.vid.start()
        else:
            if not self.vid.isOpened():
                print("Failed to open camera")
                exit()

        print("Video start")

    def getFrame(self):
        if self.type == "Pi":
            return self.vid.capture_array()
        else:
            ret, frame = self.vid.read()

            cv.waitKey(1)
            return frame

    def release(self):
        self.vid.release()

def detectSIFT(frame):
    kp, desc = sift.detectAndCompute(frame, None) # https://amroamroamro.github.io/mexopencv/matlab/cv.SIFT.detectAndCompute.html
    best = 0
    bestID = -1
    bestMatches = False
    if len(desc) == 0:
        return False
    for i in range(constants.TARGET_NUM):
        ratio = constants.RATIO_TEST_VALUES[i]
        matches = bf.knnMatch(siftDesc[i], desc, k=2) # https://docs.opencv.org/3.4/dc/dc3/tutorial_py_matcher.html
        validMatches = []
        avr = 0
        if len(matches) != 0 and len(matches[0]) == 2:
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
    ret, thresh = cv.threshold(frame, constants.CONTOUR_VALUE_THRESHOLD, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    validContours = []
    for i in range(len(contours)):
        contour = contours[i]

        #check area
        if cv.contourArea(contour) < constants.CONTOUR_AREA_MIN_THRESHOLD:
            continue

        #check solidity
        hull = cv.convexHull(contour)
        if cv.contourArea(hull) == 0:
            continue
        solidity = abs(cv.contourArea(contour) / cv.contourArea(hull))
        if abs(solidity - 1) > constants.SOLIDITY_MAX_THRESHOLD:
            continue

        #check approx
        epsilon = constants.CONTOUR_POLYDP_EPSILON * cv.arcLength(contour, True)
        approx = cv.approxPolyDP(contour, epsilon, True)
        if abs(4-len(approx)) > constants.SIDE_NUM_THRESHOLD:
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

    north_m = north * constants.FEET_TO_METERS
    east_m = east * constants.FEET_TO_METERS
    down_m = down * constants.FEET_TO_METERS

    R = constants.EARTH_RADIUS

    d_lat = north_m / R
    d_lon = east_m / (R * math.cos(math.radians(start_lat)))

    final_lat = start_lat + (math.degrees(d_lat))
    final_lon = start_lon + (math.degrees(d_lon))
    final_alt = start_alt - down_m

    return LocationGlobal(final_lat, final_lon, final_alt)

def calculate_target_gps(vehicle, dx_dist, dy_dist, dx, dy): # function takes in dx dy feet and gives new GPS coordinates
    screen_angle = math.degrees(math.atan2(dy, dx))
    global_angle_deg = -screen_angle + 90 + vehicle.heading
    global_angle_rad = math.radians(global_angle_deg)

    total_dist_ft = math.sqrt(dx_dist**2 + dy_dist**2)
    
    north_ft = total_dist_ft * math.cos(global_angle_rad)
    east_ft = total_dist_ft * math.sin(global_angle_rad)

    north_m = north_ft * constants.FEET_TO_METERS
    east_m = east_ft * constants.FEET_TO_METERS

    R = constants.EARTH_RADIUS

    vehicle_lat = vehicle.location.global_frame.lat
    vehicle_lon = vehicle.location.global_frame.lon
    
    d_lat = north_m / R
    d_lat_deg = math.degrees(d_lat)
    
    d_lon = east_m / (R * math.cos(math.radians(vehicle_lat)))
    d_lon_deg = math.degrees(d_lon)
    
    target_lat = vehicle_lat + d_lat_deg
    target_lon = vehicle_lon + d_lon_deg
    
    return target_lat, target_lon

def arm_takeoff(vehicle, altitude): # in feet
    convertedAlt = altitude * constants.FEET_TO_METERS # in meters

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
        if vehicle.location.global_relative_frame.alt >= convertedAlt * constants.ALTITUDE_THRESHOLD:
            print(f"Reached target altitude.")
            break
        time.sleep(1)

    return

def land(vehicle):
   vehicle.mode = VehicleMode("LAND")
   vehicle.armed = False
   while vehicle.armed:
        print("LANDING")
        time.sleep(1)

def set_mode(vehicle, mode):
    mode = mode.upper()
    print(f"Start set to {mode}")
    vehicle.mode = VehicleMode(mode)
    while not vehicle.mode.name==mode:
        print(f"Setting {mode}")
        time.sleep(1)
    print(f"{mode} set worked: drone mode is {vehicle.mode.name}!") #checking to make sure it is set
    return