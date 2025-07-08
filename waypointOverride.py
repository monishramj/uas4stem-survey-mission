from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

vehicle = connect(ip='/dev/ttyAMA0', wait_ready=True, baud=115200)  

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

def arm_takeoff(alt, vehicle): # in feet
   
    convertedAlt = alt * 0.3048 # in meters

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

    while True:
        print(f"Altitude:{vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt>= convertedAlt * 0.95:
            print(f"Reached target altitude.")
            break
        time.sleep(1)
    
    return

def change_alt(alt, vehicle): # in feet

   alt = alt * 0.3048 # in meters

   location = vehicle.location.global_relative_frame
   new_location = location
   new_location.alt = alt
   
   vehicle.simple_goto(new_location)

   while True:
      print(f"Altitude:{vehicle.location.global_relative_frame.alt}")
      if vehicle.location.global_relative_frame.alt>= alt * 0.95:
         print(f"Reached target altitude.")
         break
      time.sleep(1)

def auto(vehicle):
   vehicle.mode = VehicleMode("AUTO")

   while not vehicle.mode.name == "AUTO":
      print("Switching to AUTO mode...")
      time.sleep(1)

def land(vehicle):
    vehicle.mode = VehicleMode("LAND")
    vehicle.armed = False
    while vehicle.armed:
        print("LANDING")
        time.sleep(1)

def printfile(aFileName):
    print("\nMission file: %s" % aFileName)
    with open(aFileName) as f:
        for line in f:
            print(' %s' % line.strip()) 

# arm_takeoff(altitude=20, vehicle=vehicle)
# ! WAYPOINT SHOULD HAVE TAKEOFF IN MISSIONPLANNER
auto(vehicle)

# download waypoint as file
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

export_mission_filename = "downloaded_mission.txt"

output = 'QGC WPL 110\n'

home = vehicle.home_location
if home is None:
    # Fallback in case home_location is not set yet
    home = vehicle.location.global_frame
output += "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
    0, 1, 0, 16, 0, 0, 0, 0, home.lat, home.lon, home.alt, 1
)

for cmd in cmds:
    commandline = "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (
        cmd.seq, cmd.current, cmd.frame, cmd.command,
        cmd.param1, cmd.param2, cmd.param3, cmd.param4,
        cmd.x, cmd.y, cmd.z, cmd.autocontinue
    )
    output += commandline

with open(export_mission_filename, 'w') as file_:
    print("Writing downloaded mission to file...")
    file_.write(output)

# print waypoint as file to check if correctly connected
printfile(export_mission_filename)

wp_num = 2 # which waypoint to change alt at
new_alt = 30 # alt to change to

while True:
   next_wp = vehicle.commands.next
   print(f"Current waypoint: {next_wp}")
   if next_wp == wp_num:
      print(f"Waypoint {wp_num} reached. Executing maneuver")
      og_alt = vehicle.location.global_relative_frame.alt
      change_alt(new_alt, vehicle)
      print("Returning to original altitude...")
      change_alt(og_alt, vehicle)
      print("Maneuver complete. Resuming AUTO mission.")
      auto(vehicle)
   time.sleep(1)


