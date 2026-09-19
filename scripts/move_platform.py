#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import time

drone_ready = False

def model_states_callback(msg):
    global drone_ready
    if not drone_ready and 'iris' in msg.name and 'husky' in msg.name:
        try:
            drone_idx = msg.name.index('iris')
            husky_idx = msg.name.index('husky')
            
            drone_x = msg.pose[drone_idx].position.x
            drone_y = msg.pose[drone_idx].position.y
            drone_z = msg.pose[drone_idx].position.z
            
            husky_x = msg.pose[husky_idx].position.x
            husky_y = msg.pose[husky_idx].position.y
            
            # Dron Husky'nin ustunde mi? (yatay mesafe < 2.0m ve irtifa > 3.5m)
            dx = abs(drone_x - husky_x)
            dy = abs(drone_y - husky_y)
            
            if drone_z > 3.5 and dx < 2.0 and dy < 2.0:
                drone_ready = True
                rospy.loginfo("Dron aracin ustunde (z={:.1f}m, dx={:.1f}m, dy={:.1f}m). Platform harekete geciyor!".format(drone_z, dx, dy))
        except Exception as e:
            pass

from gazebo_msgs.msg import ModelState

def move_with_speed(speed):
    rospy.init_node('robot_speed_control', anonymous=True)
    rospy.loginfo("Robot hareket node baslatildi")
    
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    
    # Bypass ROS controllers and set model state directly in Gazebo for perfectly smooth movement
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.loginfo("Gazebo set_model_state yayinci olusturuldu")

    # Dronun kalkmasini bekle
    rospy.loginfo("Dronun havalanmasi ve aracin ustune gelmesi bekleniyor...")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not drone_ready:
        rate.sleep()

    if rospy.is_shutdown():
        return

    rospy.loginfo("Platform hareket basliyor (Smooth Gazebo State)!")

    state_cmd = ModelState()
    state_cmd.model_name = 'husky'
    state_cmd.twist.linear.x = speed
    state_cmd.reference_frame = 'world'
    
    start_time = time.time()
    
    loop_rate = rospy.Rate(50)
    
    while not rospy.is_shutdown() and time.time() - start_time < 1000:
        pub.publish(state_cmd)
        loop_rate.sleep()

    # Robotu durdur
    state_cmd.twist.linear.x = 0.0
    pub.publish(state_cmd)
    rospy.loginfo("Robot durduruldu")

try:
    move_with_speed(0.40)
except rospy.ROSInterruptException:
    pass
