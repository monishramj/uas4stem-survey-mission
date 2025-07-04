from dronekit import connect, VehicleMode, mavutil
import time

targetAlt = 50

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True,
                  baud=115200)  # baud rate changed from 57600

vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.mode.name == 'GUIDED' and not vehicle.armed:
    print("Arming")
    time.sleep(1)

print("Armed")

print("Taking off")
vehicle.simple_takeoff(targetAlt)
startTime = time.time()
while time.time() - startTime < 5:
    print(time.time() - startTime)
    if vehicle.location.global_Relative_Frame.alt >= targetAlt * 0.95:
        print("Reached target altitude")
        break
    # print("Altitude: " + vehicle.location.global_relative_frame.alt)
    # if vehicle.location.global_relative_frame.alt >= targetAlt*0.95:
    #     print("Reached target altitude")
    #     break
    #     time.sleep(1)

# print("Landing")
# msg = vehicle.message_factory.command_long_encode(
#     1, 0,    # target_system, target_component
#     mavutil.mavlink.MAV_CMD_NAV_LAND, #command
#     0, #confirmation
#     0, 0, 0, 0,    # param 1-4, not used
#     0, 0, # param 5-6, landing coordinate (0 means land at current location)
#     0) # param 7, altitude (not used)

# # send command to vehicle
# vehicle.send_mavlink(msg)

# startTime = time.time()
# while time.time() - startTime < 5:
#     print("Altitude: " + vehicle.location.global_relative_frame.alt)
#     print(time.time() - startTime)
#     vehicle.send_mavlink(msg)
#     if vehicle.location.global_relative_frame.alt <= 0.25:
#         print("Landed")
#         break
#         time.sleep(1)

# exiting guided
vehicle.mode = VehicleMode("LAND")
vehicle.armed = False
while vehicle.armed:
    print("LANDING")
    time.sleep(1)

print("DONE")
