#! /usr/bin/env python

import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from project.srv import FindWall, FindWallRequest
import actionlib
from project.msg import OdomRecordAction, OdomRecordGoal
from std_msgs.msg import Empty


class wall_following(object):
    def __init__(self):
        ####service####
        rospy.wait_for_service('/find_wall')  # wait till the service is available
        FindWall_service = rospy.ServiceProxy('/find_wall', FindWall)  # connect to the service
        result = FindWall_service(FindWallRequest())  # send the request
        print(result)
    
        ####acttion####
        self.PENDING = 0
        self.ACTIVE = 1
        self.DONE = 2
        self.WARN = 3
        self.ERROR = 4

        action_server_name = 'record_odom'
        self.action_client = actionlib.SimpleActionClient( action_server_name, OdomRecordAction)  # establish the action client
        rospy.loginfo('Waiting for action Server '+action_server_name)
        self.action_client.wait_for_server()
        rospy.loginfo('Action Server Found...'+action_server_name)
        # send empty goal to start the action
        self.action_client.send_goal(Empty(), feedback_cb=self.feedback_callback)
        # initially it will be zero means pending
        state_result = self.action_client.get_state()
        rospy.loginfo("state_result: "+str(state_result))
    
        ####wall_following####
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.move = Twist()
        rospy.loginfo("start wall fllowing")
    
        # subscriber
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.wait_for_message("/scan", LaserScan)
    
    def feedback_callback(self,feedback):
        print("feedback from action server:\ntotall distance moved so far\n"+str(feedback))

    def callback(self,msg):
    
        rospy.loginfo("the distance in front of robot"+str(msg.ranges[360]))
    
        if msg.ranges[360] > 0.5:
           # perform wallfollower programm
            if msg.ranges[180] > 0.3:
                # approach the wall slowly
                self.move.linear.x = 0.06
                self.move.angular.z = -0.1
                self.pub.publish(self.move)
    
            elif msg.ranges[180] < 0.2:
                # get away from the wall slowly
                self.move.linear.x = 0.06
                self.move.angular.z = 0.1
                self.pub.publish(self.move)
    
            elif ((msg.ranges[180]<0.3) and (msg.ranges[180]>0.2)):
                # keep moving forward
                self.move.linear.x = 0.06
                self.move.angular.z = 0
                self.pub.publish(self.move)
    
        else:
            # turn left while moving forward
            self.move.linear.x = 0.04
            self.move.angular.z = 0.25 # left
            self.pub.publish(self.move)
           

        state_result = self.action_client.get_state()
        if state_result == self.DONE:
            rospy.loginfo("the action is done succesfully")
            rospy.loginfo(self.action_client.get_result())

    
    
    
if __name__ == "__main__":
    rospy.init_node("project_node")  # start the node
    wall_following()
    rospy.spin()
