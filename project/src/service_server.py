#! /usr/bin/env python

import rospy
import time
from project.srv import FindWall, FindWallResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class my_node(object):

    def __init__(self):

        self.sub_s = rospy.Subscriber("/scan", LaserScan, self.callback_msg)
        rospy.wait_for_message("/scan", LaserScan)
        self.pub_s = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.FindWall_service = rospy.Service(
            '/find_wall', FindWall, self.callback)  # create the service
        self.move_2 = Twist()

    def callback_msg(self, msg):

        self.left = msg.ranges[540]  # left
        self.right = msg.ranges[180]  # right
        self.front = msg.ranges[360]  # front
        self.ready_point = msg.ranges[270]

    def callback(self, request):

        r = rospy.Rate(8)
        rospy.loginfo("left distance"+str(self.left))
        rospy.loginfo("right distance"+str(self.right))
        rospy.loginfo("front distnace"+str(self.front))

        while ((self.left == 0) and (self.right == 0) and (self.front == 0)):
            # wait until there is a value received
            rospy.sleep(1)

        rospy.loginfo('findwall service started...')

        # if left is the smaller distance
        if ((self.left < self.right) and (self.left < self.front)):

            rospy.loginfo("first case")
            left_1 = round(self.left, 1)
            front_1 = round(self.front, 1)
            # rotate untill the front of the robot faces the wall
            while front_1 != left_1:
                rospy.loginfo('rotating left..')
                self.move_2.linear.x = 0
                self.move_2.angular.z = 0.25  # rotate to the left
                self.pub_s.publish(self.move_2)
                front_1 = round(self.front, 1)
                rospy.loginfo("current left distance"+str(left_1))
                rospy.loginfo("current front distance"+str(front_1))
                r.sleep()
            # approach the wall
            while self.front >= 0.3:
                rospy.loginfo("moving forward")
                self.move_2.linear.x = 0.1
                self.move_2.angular.z = 0
                self.pub_s.publish(self.move_2)
                r.sleep()
            # rotate untill the ready point is reached
            while self.ready_point > 0.3:
                rospy.loginfo("trying to reach the start state (ready_point)")
                self.move_2.linear.x = 0
                self.move_2.angular.z = 0.25  # rotate to the left
                self.pub_s.publish(self.move_2)
                r.sleep()
        # if right is the smaller distance
        elif ((self.right < self.left) and (self.right < self.front)):

            rospy.loginfo("second case")
            right_1 = round(self.right, 1)
            front_1 = round(self.front, 1)
            # rotate untill the front of the robot faces the wall
            while front_1 != right_1:
                rospy.loginfo("rotating right ..")
                self.move_2.linear.x = 0
                self.move_2.angular.z = -0.25  # rotate to the  right
                self.pub_s.publish(self.move_2)
                front_1 = round(self.front, 1)
                rospy.loginfo("current right distance"+str(right_1))
                rospy.loginfo("current front distance"+str(front_1))
                r.sleep()
            # approach the wall
            while self.front >= 0.3:

                rospy.loginfo("moving forward")
                self.move_2.linear.x = 0.08
                self.move_2.angular.z = 0
                self.pub_s.publish(self.move_2)
                r.sleep()
            # rotate untill the ready point is reached
            while self.ready_point > 0.3:
                rospy.loginfo("trying to reach the start state (ready_point)")
                self.move_2.linear.x = 0
                self.move_2.angular.z = 0.25  # rotate to the left
                self.pub_s.publish(self.move_2)
                r.sleep()

        # fornt is the smaller distance
        else:
            rospy.loginfo("third case")
            # approach the wall
            while self.front >= 0.3:
                rospy.loginfo("moving forward")
                self.move_2.linear.x = 0.1
                self.move_2.angular.z = 0
                self.pub_s.publish(self.move_2)
                r.sleep()
            # rotate untill the ready point is reached
            while self.ready_point > 0.3:
                rospy.loginfo("trying to reach the start state (ready_point)")
                self.move_2.linear.x = 0
                self.move_2.angular.z = 0.25  # rotate to the left
                self.pub_s.publish(self.move_2)
                r.sleep()
        # stop the robot
        self.move_2.linear.x = 0
        self.move_2.angular.z = 0
        self.pub_s.publish(self.move_2)

        rospy.loginfo('findwall service finished..')
        message = FindWallResponse()
        message.wallfound = True
        return message


if __name__ == '__main__':
    rospy.init_node('service_node')
    my_object = my_node()
    rospy.spin()
