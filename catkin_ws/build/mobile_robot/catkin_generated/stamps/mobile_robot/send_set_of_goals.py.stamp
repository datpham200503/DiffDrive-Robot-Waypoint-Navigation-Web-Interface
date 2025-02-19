#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32MultiArray

x = [1.075, -0.045, -1.3, -1.72, -0.43, 0.855, 0.55, -0.725, -1.9]
y = [2.79, 3.19, 3.73, 2.99, 2.5, 1.97, 1.12, 1.68, 2.11]

x_pose = []
y_pose = []

path_received = False

def send_goals():
    global x_pose, y_pose, path_received

    rospy.init_node('navigation_goals')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    rospy.Subscriber('path', Int32MultiArray, callback)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()   

    while not rospy.is_shutdown():
        if not path_received:
            rospy.loginfo("Waiting for path data...")
            rospy.sleep(1)
            continue

        for i in range(len(x_pose)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = x_pose[i]
            goal.target_pose.pose.position.y = y_pose[i]
            goal.target_pose.pose.orientation.w = 1.0

            rospy.loginfo("Sending goal %d" % (i+1))
            client.send_goal(goal)

            client.wait_for_result()

            if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Hooray, the base moved to goal")
            else:
                rospy.loginfo("The base failed to move to goal for some reason")

        rospy.loginfo("Finished current path. Waiting for new path data...")
        path_received = False
        x_pose = [] 
        y_pose = []  
        rospy.sleep(1)

def callback(data):
    global path_received, x_pose, y_pose

    if path_received:
        rospy.logwarn("Path already received. Ignoring further updates.")
        return

    rospy.loginfo("Received path data")
    array_data = data.data

    for point in array_data:
        x_pose.append(x[point-1])
        y_pose.append(y[point-1])

    path_received = True

if __name__ == '__main__':
    try:
        send_goals()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
