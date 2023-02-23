from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil

vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, baud=38400)
vehicle.wait_ready(True, raise_exception = False)


the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
the_connection.wait_heartbeat()
print('mavutil heartbeat received')

def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize")
        time.sleep(1)

    print("Arming motors")

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(altitude)

    while True:
        print("".format(vehicle.location.global_relative_frame.alt))
        if vehicle.location.global_relative_frame.alt <= altitude*0.25:
            print("Reached target altitude")
            break
        time.sleep(1)

#arm_and_takeoff(10)

#time.sleep(10)


def maneuver():
    vehicle.mode = VehicleMode("GUIDED")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)


    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)


    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

    time.sleep(20)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)


    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 45, 50, 1, 1, 0, 0, 0)


    time.sleep(20)

    #msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    #print(msg)


def land():
    vehicle.mode = VehicleMode("LAND")


maneuver()

vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, baud=38400)
vehicle.wait_ready(True, raise_exception = False)
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
the_connection.wait_heartbeat()
print('mavutil heartbeat received')

land()


"""
condition_yaw(360)
time.sleep(10)
condition_yaw(-180)
time.sleep(10)

"""
#print('Landing Time')

#vehicle.mode = VehicleMode('LAND')
#vehicle.close()

