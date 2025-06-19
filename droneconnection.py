from dronekit import connect, mavutil

vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

msg = vehicle.message_factory.command_long_encode(
  1, 1,
  mavutil.mavlink.STATUSTEXT,  # msg
  0, # confirmation 
  mavutil.mavlink.MAV_SEVERITY_INFO, 
  'hello world', 0, 0, # ????
  0, 0,
  0)

vehicle.send_mavlink(msg)

response = vehicle.recv_match(type='COMMAND_ACK', blocking=True)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Command accepted")
else:
    print("Command failed")

# from pymavlink import mavutil

# testmsg = 'Hello world!' # max 50 char
# connection = mavutil.mavlink_connection('', baud=0)
# connection.wait_heartbeat()


# print("Heartbeat received from system (system %u component %u)" % (connection.target_system, connection.target_component))


# connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, testmsg.encode("utf-8"))
# print(f"Message sent: {testmsg}")
