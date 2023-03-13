import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
import PIL
from cv_bridge import CvBridge
from ultralytics import YOLO
import sys
from dronekit import connect, VehicleMode
import time
from pymavlink import mavutil


def displayImage(msg):
    convertedImage = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    
    # cv2 array format -> height, width, number of channels
    frameHcentre = convertedImage.shape[1]//2

    r = model.predict(convertedImage)

    # x and y are the coordinates of the centre of the box
    params = list(r)[0].boxes.xywh[0]
    x = float(params[0])
    y = float(params[1])
    w = float(params[2])
    h = float(params[3])
    cv2.rectangle(convertedImage, (int(x - (w//2)), int(y - (h//2))), (int(x + (w//2)), int(y + (h//2))), (255,0,0), 2)
    reconvertedImage = PIL.Image.fromarray(np.uint8(convertedImage), 'RGB')

    boxCentre = x
    print('frameHcentre: {}'.format(frameHcentre))
    print('boxCentre: {}'.format(boxCentre))

    #cv2.imwrite('resultimage.png',convertedImage)
    smsg = Image()
    smsg.header.stamp = rospy.Time.now()
    smsg.height = reconvertedImage.height
    smsg.width = reconvertedImage.width
    smsg.encoding = 'rgb8'
    smsg.is_bigendian = False
    smsg.step = 3 * reconvertedImage.width
    smsg.data = convertedImage.tobytes()

    pub.publish(smsg)
   
    if frameHcentre < boxCentre:
        vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, baud=38400)
        vehicle.wait_ready(True, raise_exception = False)
        the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
        the_connection.wait_heartbeat()
        print('mavutil heartbeat received')
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 15, 50, 1, 1, 0, 0, 0)
        #time.sleep(20)

    elif frameHcentre > boxCentre:
        vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, baud=38400)
        vehicle.wait_ready(True, raise_exception = False)
        the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
        the_connection.wait_heartbeat()
        print('mavutil heartbeat received')
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, 15, 50, -1, 1, 0, 0, 0)
        #time.sleep(20)
   

model = YOLO('yolov8s.pt')

##### Initiate drone ###########
vehicle = connect('udp:127.0.0.1:14550', wait_ready=False, baud=38400)
vehicle.wait_ready(True, raise_exception = False)
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
the_connection.wait_heartbeat()
print('mavutil heartbeat received')
vehicle.mode = VehicleMode("GUIDED")

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

ackmsg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(ackmsg)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
time.sleep(10)
#################################

rospy.init_node('getCameraFeed', anonymous = True)
pub = rospy.Publisher('detectedobjects', Image, queue_size = 10)
sub = rospy.Subscriber("/webcam/image_raw", Image, displayImage)
bridge = CvBridge()
rospy.spin()

