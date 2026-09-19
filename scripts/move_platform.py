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

def move_with_speed(speed):
    rospy.init_node('robot_speed_control', anonymous=True)
    rospy.loginfo("Robot hareket node baslatildi")
    
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Robot cmd_vel yayinci olusturuldu")

    # Dronun kalkmasini bekle
    rospy.loginfo("Dronun havalanmasi ve aracin ustune gelmesi bekleniyor...")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not drone_ready:
        rate.sleep()

    if rospy.is_shutdown():
        return

    rospy.loginfo("Platform hareket basliyor!")

    # Twist mesajı oluşturma
    move_cmd = Twist()
    move_cmd.linear.x = speed  # Hızı ayarla
    move_cmd.angular.z = 0.0 # Dönüş olmadan düz hareket
    
    start_time = time.time()
    while not rospy.is_shutdown() and time.time() - start_time < 1000:
        rospy.loginfo(f"Robot hareket ediyor {speed} m/s")
        pub.publish(move_cmd)
        rate.sleep()

    # Robotu durdur
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Robot durduruldu")

try:
    move_with_speed(0.20)
except rospy.ROSInterruptException:
    pass
