#!/usr/bin/python
##########IMPORTS##########

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco
import time
import math
import numpy as np
import ros_numpy as rnp
from dronekit import connect, VehicleMode
from pymavlink import mavutil


bridge=CvBridge()

vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)
vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 30

velocity = .5
takeoff_height = 4

##########VARIABLES##########

newimg_pub = rospy.Publisher('/webcam/image_raw', Image, queue_size=10)


id_to_find = 72 ##arucoID
marker_size = 0.2 ##CM

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()

horizontal_res = 640
vertical_res = 480

horizontal_fov = 68.75 * (math.pi / 180)  # 68.75 dereceyi radiana dönüştür
vertical_fov = 54.32 * (math.pi / 180)    # 54.32 dereceyi radiana dönüştür


found_count = 0
notfound_count = 0

time_last = 0
time_to_wait = .1
############CAMERA INTRINSICS#############

np_camera_matrix =np.array([
        [467.74270306499267, 0.0, 320.5],
        [0.0, 467.74270306499267, 240.5],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
np_dist_coeff = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

############FUNCTIONS#############

def arm_and_takeoff(targetHeight):
    rospy.loginfo("Pre-arm checks...")
    while not vehicle.is_armable:
        rospy.loginfo(" Waiting for vehicle to initialize...")
        time.sleep(1)

    rospy.loginfo("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        rospy.loginfo(" Waiting for arming...")
        time.sleep(1)

    rospy.loginfo("Taking off!")
    vehicle.simple_takeoff(targetHeight)

    while True:
        rospy.loginfo("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= targetHeight * 0.95:
            rospy.loginfo("Target altitude reached!")
            break
        time.sleep(1)

    return None

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,0,0,0,0)
    vehicle.send_mavlink(msg)
    rospy.loginfo('Velocity command sent!')
    vehicle.flush()

def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        x,
        y,
        0,0,0)
    vehicle.send_mavlink(msg)
    rospy.loginfo('Land message sent!')
    vehicle.flush()



def msg_receiver(message):
    global notfound_count, found_count, time_last, time_to_wait, id_to_find
    

    if time.time() - time_last > time_to_wait:
        #webcam image data

        np_data = rnp.numpify(message) ##Deserialize image data into array
        gray_img = cv2.cvtColor(np_data, cv2.COLOR_BGR2GRAY)
        ids = ''
        (corners, ids, rejected) = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

        try:
            if ids is not None:
                rospy.loginfo(f'Found marker with ID: {ids}')
                if ids[0]==id_to_find:
                    ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix=np_camera_matrix, distCoeffs=np_dist_coeff)
                    (rvec, tvec) = (ret[0][0,0,:], ret[1][0,0,:])
                    x = '{:.2f}'.format(tvec[0])
                    y = '{:.2f}'.format(tvec[1])
                    z = '{:.2f}'.format(tvec[2])

                    x_sum = 0
                    y_sum = 0

                    x_sum = corners[0][0][0][0] + corners[0][0][1][0] + corners[0][0][2][0] + corners[0][0][3][0]
                    y_sum = corners[0][0][0][1] + corners[0][0][1][1] + corners[0][0][2][1] + corners[0][0][3][1]

                    x_avg = x_sum / 4
                    y_avg = y_sum / 4

                    x_ang = (x_avg - horizontal_res*.5)*horizontal_fov/horizontal_res
                    y_ang = (y_avg - vertical_res*.5)*vertical_fov/vertical_res

                    if vehicle.mode != 'LAND':
                        vehicle.mode = VehicleMode('LAND')
                        while vehicle.mode != 'LAND':
                            time.sleep(1)
                        rospy.loginfo('Vehicle in LAND mode!')
                        send_land_message(x_ang, y_ang)
                    else:
                        send_land_message(x_ang, y_ang)

                    marker_position = 'MARKER POSITION: x='+x+' y='+y+' z='+z
                    rospy.loginfo('FOUND COUNT: ' +str(found_count) + ' - NOTFOUND COUNT: ' +str(notfound_count))

                    aruco.drawDetectedMarkers(np_data, corners)
                    cv2.drawFrameAxes(np_data, np_camera_matrix, np_dist_coeff, rvec, tvec, 10)
                    
                    cv2.putText(np_data, marker_position, (10,50), 0, 0.5, (255,0,0), thickness=1)

                    found_count = found_count + 1

                    estimated_altitude = tvec[2]
                    cv2.putText(np_data, 'Tahmini Yukseklik: '+str(estimated_altitude), (10,90), 0, 0.5, (0,0,255), thickness=1)
                                  
                else:
                    notfound_count = notfound_count + 1
                
            else:

                notfound_count = notfound_count + 1

        except Exception as e:
            rospy.loginfo('Target likely not found!')
            print(e)
            notfound_count = notfound_count + 1
        new_msg = rnp.msgify(Image, np_data, encoding='rgb8')
        newimg_pub.publish(new_msg)
        cv2.waitKey(1)   
        time_last = time.time()
    else:
        return None
                
def disarm_on_landing():
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        
        # İniş tamamlandığında motorları kapat
        if altitude <= 0.36:  # Altitude threshold
            rospy.loginfo("Landing complete. Disarming motors...")
            vehicle.armed = False
            while vehicle.armed:
                rospy.loginfo("Waiting for motors to disarm...")
                time.sleep(1)
            rospy.loginfo("Motors disarmed.")
            break
        time.sleep(0.5)

def subscriber():
    rospy.init_node('drone_node', anonymous=False)
    sub = rospy.Subscriber('/webcam/image_raw', Image, msg_receiver)
    rospy.spin()

if __name__=='__main__':
    try:
        arm_and_takeoff(takeoff_height)
        time.sleep(1)
        send_local_ned_velocity(velocity, 0, 0)
        time.sleep(1)
        subscriber()
        disarm_on_landing()

    except rospy.ROSInterruptException:
        pass
