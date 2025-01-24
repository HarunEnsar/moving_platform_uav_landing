#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def move_with_speed(speed):
    rospy.init_node('robot_speed_control', anonymous=True)
    rospy.loginfo("Robot hareket node başlatıldı")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Robot cmd_vel yayıncı oluşturuldu")

    # Twist mesajı oluşturma
    move_cmd = Twist()
    move_cmd.linear.x = speed  # Hızı ayarla
    move_cmd.angular.z = 0.0 # Dönüş olmadan düz hareket
    time.sleep(4)  # 4 saniye bekle
    # 10 saniye boyunca mesaj yayınla
    rate = rospy.Rate(10)  # 10 Hz
    start_time = time.time()
    while time.time() - start_time < 1000:
        rospy.loginfo(f"Robot hareket ediyor {speed} m/s")
        pub.publish(move_cmd)
        rate.sleep()

    # Robotu durdur
    move_cmd.linear.x = 0.0
    pub.publish(move_cmd)
    rospy.loginfo("Robot durduruldu")


try:
    move_with_speed(0.25)
except rospy.ROSInterruptException:
    pass
