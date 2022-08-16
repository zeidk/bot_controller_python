from custom_msgs.msg import MoveBotAction, MoveBotFeedback, MoveBotResult, MoveBotActionResult
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt
from math import atan2, pi
import actionlib
import rospy

class MoveBotActionServer:

    def __init__(self):
        rospy.init_node('move_bot_server')
        self._feedback = MoveBotFeedback()
        self._result = MoveBotResult()
        self._velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, self.odom_cb)

        #  gains for the proportional controller
        self._kp_linear = 0.2
        self._kp_angular = 0.2

        #  default velocities for going in a straight line
        self._velocity_msg = Twist()
        self._velocity_msg.linear.x = 0.1
        self._velocity_msg.angular.z = 0.1

        # current pose of the robot
        self._current_pose = None
        self._current_x_pos = None
        self._current_y_pos = None
        self._current_orientation = None
        
        self._rate = rospy.Rate(10)
        # Set up the action server
        self._action_server = actionlib.SimpleActionServer(
            'move_bot_action_server',
            MoveBotAction,
            execute_cb=self.action_server_cb,
            auto_start=False)
        # Start the action server
        self._action_server.start()

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
        

    def odom_cb(self, msg: Odometry):
        """
        Callback function for the Topic odom
        Args:
            msg (nav_msgs/Odometry): Odometry message
        """
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        self._current_x_pos = msg.pose.pose.position.x
        self._current_y_pos = msg.pose.pose.position.y
        self._current_orientation = euler_from_quaternion(quaternion)
        
        
    def action_server_cb(self, goal):
        """
        Make the robot reach a 2D goal using a proportional controller
        Args:
            goal_x (float): x position
            goal_y (float): y position
        """
        
        success = True
        
        goal_x = goal.x
        goal_y = goal.y
        rospy.loginfo("Go to goal ({}, {})".format(goal_x, goal_y))

        # get position and yaw from transform
        while self._current_x_pos is None:
            rospy.sleep(0.1)

        distance_to_goal = MoveBotActionServer.compute_distance(
            self._current_x_pos, self._current_y_pos, goal_x, goal_y)

        while not rospy.is_shutdown():
            if self._action_server.is_preempt_requested():
                rospy.loginfo('move_bot_action_server: Preempted')
                self._action_server.set_preempted()
                success = False
                move_cmd = Twist()
                move_cmd.linear.x = 0
                move_cmd.linear.z = 0
                self._velocity_publisher.publish(move_cmd)
                break
  
            move_cmd = Twist()
            if distance_to_goal > 0.1:
                distance_to_goal = MoveBotActionServer.compute_distance(self._current_x_pos,
                                                                  self._current_y_pos, goal_x, goal_y)
                
                # update the feedback
                self._feedback.distance_to_goal = " " + str(distance_to_goal)
                # publish the feedback
                self._action_server.publish_feedback(self._feedback)
                
                # compute the heading
                angle_to_goal = atan2(
                    goal_y - self._current_y_pos, goal_x - self._current_x_pos)

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
                self._rate.sleep()
            else:
                move_cmd = Twist()
                move_cmd.linear.x = 0
                move_cmd.linear.z = 0
                self._velocity_publisher.publish(move_cmd)
                break
                
        if success:
            self._result.result = "Goal Reached"
            self._action_server.set_succeeded(self._result)