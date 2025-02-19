#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32MultiArray

def publish_array():
    rospy.init_node('array_publisher', anonymous=True)
    pub = rospy.Publisher('array_topic', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(0.2)

    data = [1, 2, 4, 5, 6]
    msg = Int32MultiArray()
    msg.data = data

    while not rospy.is_shutdown():
        rospy.loginfo("Publishing data: %s", data)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_array()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node end.")
