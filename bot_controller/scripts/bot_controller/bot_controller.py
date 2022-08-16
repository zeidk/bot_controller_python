#!/usr/bin/env python3

import rospy
import sys
import tf

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, atan2, pi
from geometry_msgs.msg import Twist
from custom_msgs.msg import RobotStatus


class BotController(object):
    """
    A controller class to drive a turtlebot in Gazebo.
    """

    def __init__(self, rate=10):
        rospy.init_node('bot_controller')
        rospy.loginfo('Press Ctrl c to exit')

        #  used to check whether a camera data has been processed
        self._rate = rospy.Rate(rate)
        self._velocity_msg = Twist()
        #  gains for the proportional controller
        self._kp_linear = 0.2
        self._kp_angular = 0.2

        #  default velocities for going in a straight line
        self._velocity_msg.linear.x = 0.1
        self._velocity_msg.angular.z = 0.1
        
        # current velocities of the robot
        self._linear_x = None
        self._angular_z = None

        # current pose of the robot
        self._current_pose = None
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        self._initial_orientation = None

        #  used to check whether the goal has been reached
        self._goal_reached = False

        # Publishers
        self._velocity_publisher = rospy.Publisher(
            'cmd_vel', Twist, queue_size=10)
        self._robot_status_publisher = rospy.Publisher(
            'robot_status', RobotStatus, queue_size=10)
        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # rospy.Subscriber("/robot_status", RobotStatus, self.robot_status_callback)

        self.handle_inputs()
        rospy.spin()

    @staticmethod
    def compute_distance(x1, y1, x2, y2):
        """Compute distance between 2 points

        Args:
            x1 (float): x position of the first point
            y1 (float): y position of the first point
            x2 (float): x position of the second point
            y2 (float): y position of the second point

        Returns:
            float: distance between 2 points
        """
        return sqrt(((x2-x1)**2) + ((y2-y1)**2))
    

    def robot_status_callback(self, msg: RobotStatus):
        """
        Callback method for the Topic /robot_status

        Args:
            msg (RobotStatus): One RobotStatus Message
        """
        rospy.loginfo(msg.DRIVE_TYPE)


    def odom_callback(self, msg: Odometry):
        """
        Callback function for the Topic odom
        Args:
            msg (nav_msgs/Odometry): One Odometry message
        """
        # self._current_pose = msg.pose.pose
        # self._linear_x = msg.twist.twist.linear.x
        # self._angular_z = msg.twist.twist.angular.z
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self._current_x_pos = msg.pose.pose.position.x
        self._current_y_pos = msg.pose.pose.position.y
        self._current_orientation = euler_from_quaternion(quaternion)


    def go_to_goal(self, goal_x, goal_y):
        """
        Make the robot reach a 2D goal using a proportional controller
        Args:
            goal_x (float): x position
            goal_y (float): y position
        """
        rospy.loginfo("Go to goal ({}, {})".format(goal_x, goal_y))

        while self._current_x_pos is None or self._current_x_pos is None:
            rospy.sleep(1)
            
        distance_to_goal = BotController.compute_distance(
            self._current_x_pos, self._current_y_pos, goal_x, goal_y)

        while not rospy.is_shutdown():
            move_cmd = Twist()
            if distance_to_goal > 0.1:
                distance_to_goal = BotController.compute_distance(self._current_x_pos,
                                                                  self._current_y_pos, goal_x, goal_y)
                # compute the heading
                angle_to_goal = atan2(
                    goal_y - self._current_y_pos, goal_x - self._current_x_pos)

                # rospy.loginfo("Distance to goal: {}".format(distance_to_goal))
                # rospy.loginfo("Angle to goal: {}".format(angle_to_goal))

                # Make the robot rotate to face the goal
                if angle_to_goal < 0:
                    angle_to_goal = 2 * pi + angle_to_goal

                # compute relative orientation between robot and goal
                w = angle_to_goal - self._current_orientation[2]
                if w > pi:
                    w = w - 2 * pi

                # proportional control for angular velocity
                move_cmd.angular.z = self._kp_angular * w

                # turtlebot max angular velocity is 2.84 rad/s
                if move_cmd.angular.z > 0:
                    move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
                else:
                    move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

                # proportional control for linear velocity
                # turtlebot max linear velocity is 0.22 m/s
                move_cmd.linear.x = min(
                    self._kp_linear * distance_to_goal, 0.6)

                self._velocity_publisher.publish(move_cmd)
                self.publish_robot_status()
                self._rate.sleep()
            else:
                rospy.loginfo("Goal reached")
                self._goal_reached = True
                self.run(0, 0)
                return True
                # break

    def run(self, linear, angular):
        """
        Publish linear and angular velocities to cmd_vel Topic.
        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        velocity = Twist()
        velocity.linear.x = linear
        velocity.angular.z = angular
        self._velocity_publisher.publish(velocity)

    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")
        
    def publish_robot_status(self):
        """
        Method to build and publish Messages on the Topic robot_status
        """
        robot_status = RobotStatus()
        # robot_status.model = "Unknown"
        # if rospy.has_param("tb3_model"):
        #     robot_status.model = rospy.get_param("tb3_model") 
        # robot_status.pose = self._current_pose
        # robot_status.velocities[0] = self._linear_x
        # robot_status.velocities[1] = self._angular_z
        
        # if robot_status.pose is not None:
        #     if robot_status.velocities[0] is not None:
        #         if robot_status.velocities[1] is not None:
        #             # rospy.loginfo("Publishing robot status")
        #             self._robot_status_publisher.publish(robot_status)
                    

    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        # get the action to perform
        # drive straight or go to goal
        action_name = rospy.get_param("~action")

        
        if action_name == "goal":
            x = rospy.get_param("~x")
            y = rospy.get_param("~y")
            if x and y:
                if self.go_to_goal(x, y):
                    rospy.logwarn("Action completed")
                    rospy.on_shutdown(self.myhook)
                    sys.exit(1)
            else:
                rospy.logerr("x or y is missing")
                rospy.on_shutdown(self.myhook)
                sys.exit(1)
        elif action_name == "publish":
            while not rospy.is_shutdown():
                self.publish_robot_status()
            rospy.on_shutdown(self.myhook)
            sys.exit(1)
        else:
            rospy.logerr("Unknown action")
            rospy.on_shutdown(self.myhook)
            sys.exit(1)
