#! /usr/bin/env python

import math
import rospy
import actionlib
from project.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tf
import copy


def update_theta(theta):
    # theta
    orientation_ = Odometry()
    orientation_.pose.pose.orientation = theta
    quaternion = [orientation_.pose.pose.orientation.x, orientation_.pose.pose.orientation.y,
                  orientation_.pose.pose.orientation.z, orientation_.pose.pose.orientation.w]
    euler = tf.transformations.euler_from_quaternion(quaternion)
    angle = euler[2]  # Get the yaw angle from the euler angles
    return angle


class record_odomClass(object):
    _feedback = OdomRecordFeedback()
    _result = OdomRecordResult()
    odom1 = Odometry()

    def __init__(self):

        self.server = actionlib.SimpleActionServer('record_odom', OdomRecordAction, self.callback_action, False)
        rospy.sub = rospy.Subscriber('/odom', Odometry, self.callback)
        rospy.wait_for_message('/odom', Odometry)
        self.server.start()

    def callback(self, msg):
        self.odom1.pose.pose.position.x = msg.pose.pose.position.x
        self.odom1.pose.pose.position.y = msg.pose.pose.position.y
        self.odom1.pose.pose.orientation = msg.pose.pose.orientation

    def callback_action(self, goal):
        odom = Odometry()
        odom = copy.deepcopy(self.odom1)
        current_position = Point()
        last_position = Point()

        r = rospy.Rate(1)
        success = True
        distance = 0
        distance_from_home_position = 0

        home_position = Point()
        home_position.x = odom.pose.pose.position.x
        home_position.y = odom.pose.pose.position.y
        home_position.z = update_theta(odom.pose.pose.orientation)

        self._feedback.current_total = 0.0
        self. _result.list_of_odoms = []

        rospy.loginfo("OdomRecord action started")

        #x and y
        current_position.x = odom.pose.pose.position.x
        current_position.y = odom.pose.pose.position.y
        current_position.z = update_theta(odom.pose.pose.orientation)

        # store odometry
        self._result.list_of_odoms.append(current_position)

        # start_time
        t_start = rospy.Time.now()

        # last position
        last_position = copy.deepcopy(current_position)

        while(1):

            odom = copy.deepcopy(self.odom1)

            current_position.x = odom.pose.pose.position.x
            current_position.y = odom.pose.pose.position.y
            # theta
            current_position.z = update_theta(odom.pose.pose.orientation)

            #rospy.logwarn("current:   "+str(round(current_position.x, 5)))
            #rospy.logwarn("last:      "+str(round(last_position.x, 5)))
            #rospy.logwarn("home:      "+str(round(home_position.x, 5)))

            while (round(current_position.x, 5) == round(last_position.x, 5) and round(current_position.y, 5) == round(last_position.y, 5)):

                odom = copy.deepcopy(self.odom1)
                current_position.x = odom.pose.pose.position.x
                current_position.y = odom.pose.pose.position.y
                # theta
                current_position.z = update_theta(odom.pose.pose.orientation)
                #rospy.logwarn("current:   "+str(round(current_position.x, 7)))
                #rospy.logwarn("last:      "+str(round(last_position.x, 7)))
                rospy.loginfo("the robot is not moving")
                rospy.sleep(0.5)

            if self.server.is_preempt_requested():
                rospy.loginfo("the goal is cnaceled")
                self.server.set_preempted()
                success = False
                break

            else:

                distance_x = current_position.x - last_position.x
                distance_y = current_position.y - last_position.y

                # distance
                distance += abs(math.sqrt(distance_x**2 + distance_y**2))

                # publish the feedback
                self._feedback.current_total = distance
                self.server.publish_feedback(self._feedback)

                # distance from home
                distance_x_home = (current_position.x) - (home_position.x)
                distance_y_home = (current_position.y) - (home_position.y)

                distance_from_home_position = math.sqrt(
                    distance_x_home**2 + distance_y_home**2)

                # update position
                last_position = copy.deepcopy(current_position)

                # store the recorded odometery
                self._result.list_of_odoms.append(current_position)

                t_before_end = rospy.Time.now()
                # rospy.loginfo(t_before_end-t_start)
                if((t_before_end - t_start).to_sec() > 15):
                    rospy.loginfo("about to finish")
                    rospy.loginfo("distance from home is : %s",
                                  distance_from_home_position)
                    # see if the turtlebot done a complete loop
                    if (round(distance_from_home_position, 3) < 0.3):
                        rospy.loginfo(distance_from_home_position)
                        break

                r.sleep()

        if success:
            rospy.loginfo("action successd")
            print(self._result.list_of_odoms)
            self.server.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('action_node')
    record_odomClass()
    rospy.spin()
