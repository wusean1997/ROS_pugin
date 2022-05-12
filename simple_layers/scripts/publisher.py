#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import random

data_x = 0.0
data_y = 0.0
data_yaw = 0.0

def talker():
    
    global data_x, data_y, data_yaw, data
    #pub = rospy.Publisher('pedestrain/pedestrain_point', Float32, queue_size=10)
    pub = rospy.Publisher('pedestrain/pedestrain_point', Float32MultiArray, queue_size=10)
    rospy.init_node('pedestrain', anonymous=True)
    rate = rospy.Rate(0.01) # 1hz
    while not rospy.is_shutdown():
        
        data_x = random.uniform(0.0,2.0)
        data_y = random.uniform(0.0,2.0)
        data_yaw = random.uniform(0.0,360.0)
        data = Float32MultiArray()
        data.data = [data_x,data_y,data_yaw]
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
