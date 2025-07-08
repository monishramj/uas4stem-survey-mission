from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import video
import numpy as np
import cv2 as cv

sift = cv.xfeatures2d.SIFT_create()
bf = cv.BFMatcher()
templates = []
templateSizes = []
siftKP = []
siftDesc = []

# * TODO: create algorithm on coming to altitude and moving drone to center image
# 4. TEST algorithm to determine distance of movement depending on distance from center of camera on screen
# -- something haivng to do with calculating and translating vector?
# 5. move drone to that specified local coordinate
# 6. run TEST algorithm to double check center alignment of image
# 7. repeat TEST movement until centered
# 8. get gps coordinates
# 9. run opencv to determine image type
# 10. go back to initial gps and land


#! 1. connect x
vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  

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

arm_takeoff(altitude=20, vehicle=vehicle)
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

out = cv.VideoWriter('output.mp4', cv.VideoWriter_fourcc(*'MJPG'), 60.0, (640, 480))

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
        print("Found center: (" + str(dx) + ", " + str(dy) + ")")

    kp, desc = sift.detectAndCompute(filtered, None)
    if len(contours) != 0:
        detect = video.detectSIFT(filtered, contours[0])
    else:
        detect = False
    if detect != False:
        frame = cv.drawMatches(templates[detect[0]], siftKP[detect[0]], frame, kp, detect[1], None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    out.write(frame)
    cv.imshow('stream', frame)
    if cv.waitKey(1) == ord('q'):
        break;

out.release()
vid.release()
cv.destroyAllWindows()


