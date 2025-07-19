from dronekit import connect, VehicleMode, LocationGlobal, mavutil
import time, math, traceback
from enum import Enum
from datetime import datetime
import numpy as np
import cv2 as cv
import constants, utils
from picamera2 import Picamera2

# !!! from Vick Y.'s UAS4STEM 

#! 1.----- CONNECT THE DRONE -----

date_and_time = datetime.now()
print("The current date and time of flight is", date_and_time)
print("Trying to connect...")
drone = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=constants.BAUD) 
print("Connected BOOYAH!")

#! 2.----- TAKEOFF ------

utils.arm_takeoff(drone, constants.TAKEOFF_ALTITUDE)
utils.set_mode(drone, "AUTO")

#! 3.----- PICAM INIT -------

print("Begin PICAM setup")

centerX = constants.WIDTH/2
centerY = constants.HEIGHT/2
out = cv.VideoWriter('output.avi', constants.RECORDING_FOURCC, constants.RECORDING_FPS, (constants.WIDTH, constants.HEIGHT))
vid = utils.Video("Pi")
#vid = utils.Video("cam")
vid.start()


#! 4.------ MISSION INIT ------
class State(Enum):
    AUTO_SCANNING, AUTO_NOT_SCANNING, AUTO_TO_BRAKE, BRAKE, BRAKE_TO_GUIDED, GUIDED, GUIDED_TO_BRAKE, BRAKE_TO_AUTO = range(8)

state = State.AUTO_SCANNING.value
found = []
currentFound = []
dx, dy = 0, 0
startTime = time.time()
movingTime = time.time()
dist_from_center = 0


#! 5.------ SEARCH ------

try:
    while True:

        if drone.mode.name == "RTL":
            break

        #Process video
        frame = vid.getFrame()
        filtered = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        contours = utils.findPOI(filtered)
        cv.putText(frame, f'GPS: ({drone.location.global_frame.lat}, {drone.location.global_frame.lon})', (10, 30), constants.FONT, 1, (0, 255, 255), 2, cv.LINE_4)
        
        contour = None
        if len(contours) != 0:
            contour = contours[0]
            frame = cv.drawContours(frame, contours, -1, constants.CONTOUR_OUTLINE_COLOR, constants.CONTOUR_OUTLINE_THICKNESS)

            #compute pixel offset
            M = cv.moments(contour)
            targetX = int(M["m10"] / M["m00"])
            targetY = int(M["m01"] / M["m00"])
            cv.circle(frame, (targetX, targetY), 5, constants.CONTOUR_OUTLINE_COLOR, -1)
            dx = -(centerX - targetX)
            dy = centerY - targetY
            dist_from_center = math.hypot(dx, dy)

            #compute features
            detect = utils.detectSIFT(filtered)

            #pixel sidelength to realworld estimation
            epsilon = constants.CONTOUR_POLYDP_EPSILON * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)

            pts = approx.reshape(4,2)
            side1 = np.linalg.norm(pts[0] - pts[1])
            side2 = np.linalg.norm(pts[1] - pts[2])
            side3 = np.linalg.norm(pts[2] - pts[3])
            side4 = np.linalg.norm(pts[3] - pts[0])

            avg_side = (side1 + side2 + side3 + side4) / 4
            print(f"Target offset: dx={dx:.2f}, dy={dy:.2f}, dist={dist_from_center:.2f}")
            dx_dist = (dx/avg_side) * constants.TARGET_SIDE_LENGTH
            dy_dist = (dy/avg_side) * constants.TARGET_SIDE_LENGTH
            approx_dist = (dist_from_center/avg_side) * constants.TARGET_SIDE_LENGTH
            cv.putText(frame, f'dx, dy: {int(dx)}, {int(dy)}, real: {dx_dist}ft, {dy_dist}ft', (10, 70), constants.FONT, 1, (0, 255, 255), 2, cv.LINE_4)
            approx_lat, approx_lon = utils.calculate_target_gps(drone, dx_dist, dy_dist, dx, dy)
            cv.putText(frame, f'~distToTarget: {int((dist_from_center/avg_side) * 4)}ft; ({approx_lat}, {approx_lon})', (10, 110), constants.FONT, 1, (0, 255, 255), 2, cv.LINE_4)

        out.write(frame) # write out frame w/ debug lines

        print(State(state))

        match state:
            case State.AUTO_SCANNING.value:
                if len(contours) != 0:
                    state = State.AUTO_TO_BRAKE.value
                    drone.mode = VehicleMode("BRAKE")

            case State.AUTO_NOT_SCANNING.value:
                if time.time() >= startTime + constants.TARGET_COOLDOWN:
                    state = State.AUTO_SCANNING.value

            case State.AUTO_TO_BRAKE.value:
                if drone.mode.name == "BRAKE":
                    state = State.BRAKE.value
                    startTime = time.time()
                    currentFound = []
                    for i in range(constants.TARGET_NUM):
                        currentFound.append(0)

            case State.GUIDED_TO_BRAKE.value:
                if drone.mode.name == "BRAKE":
                    state = State.BRAKE.value

            case State.BRAKE.value:
                if time.time() - startTime > constants.CENTERING_TIMEOUT:
                    state = State.BRAKE_TO_AUTO.value
                    drone.mode = VehicleMode("AUTO")

                if len(contours) != 0:
                    if detect != False:
                        currentFound[detect[0]] += 1
                        target_id = detect[0]
                        target_name = constants.TARGETS[target_id]

                        if detect[0] in found:
                            print(f"Already found target {target_id}: {target_name}, ignore ts")

                            if currentFound[detect[0]] >= constants.ALREADY_FOUND_MAX_THRESHOLD:
                                state = State.BRAKE_TO_AUTO.value
                                drone.mode = VehicleMode("AUTO")
                                continue

                        print(f"New target {target_id}: {target_name} found")

                    if dist_from_center > constants.CENTER_THRESHOLD:
                        print("Try move over target")
                        state = State.BRAKE_TO_GUIDED.value
                        drone.mode = VehicleMode("GUIDED")

                        screen_angle = math.degrees(math.atan2(dy, dx))-90
                        global_angle = math.radians(-1 * screen_angle + drone.heading) # heading is calc. yaw, 0 is N, goes E i believe

                        move_north = constants.STEP_LENGTH * math.cos(global_angle)
                        move_east = constants.STEP_LENGTH * math.sin(global_angle)

                        new_loc = utils.create_local_coordinate(drone, move_north, move_east, 0)
                        print(f"Moving to: N={move_north:.2f}ft, E={move_east:.2f}ft")

                        drone.simple_goto(new_loc, groundspeed=constants.STEP_SPEED)
                    else:
                        print(f"Target {target_id} centered.")
                        print(f"GPS coordinate: lat:{drone.location.global_frame.lat}, lon:{drone.location.global_frame.lon}")

                        maxFound = max(currentFound)
                        if maxFound < constants.FOUND_MIN_THRESHOLD:
                            continue
                        found.append(currentFound.index(maxFound))
                        state = State.BRAKE_TO_AUTO.value
                        drone.mode = VehicleMode("AUTO")


            
            case State.BRAKE_TO_AUTO.value:
                if drone.mode.name == "AUTO":
                    state = State.AUTO_NOT_SCANNING.value
                    startTime = time.time()

            case State.BRAKE_TO_GUIDED.value:
                if drone.mode.name == "GUIDED":
                    state = State.GUIDED.value
                    drone.mode = VehicleMode("GUIDED")
                    guidedTime = time.time()

            case State.GUIDED.value:
                if time.time() - guidedTime > constants.MOVE_TIME:
                    state = State.GUIDED_TO_BRAKE.value
                    drone.mode = VehicleMode("BRAKE")

except KeyboardInterrupt:
    print("Mission interrupted manually")
except Exception as e:
    print(f"ERROR. Mission interrupted by: {type(e).__name__} - {e}.")
    tb = traceback.format_exc()
    print(tb)  # Or write to a log file


#! 6.------ END ------

out.release()
#vid.release()
cv.destroyAllWindows()
print("Preparing to return and land")
utils.set_mode(drone, "RTL")
print("We done WOOOHOO (or not womp)")