from pymavlink import mavutil

testmsg = 'Hello world!' # max 50 char
connection = mavutil.mavlink_connection('', baud=0)
connection.wait_heartbeat()


print("Heartbeat received from system (system %u component %u)" % (connection.target_system, connection.target_component))


connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, testmsg.encode("utf-8"))
print(f"Message sent: {testmsg}")
