

import rospy
import actionlib
import sys

from custom_msgs.msg import MoveBotAction, MoveBotGoal


class MoveBotActionClient(object):
    def __init__(self):
        rospy.init_node('move_robot_client')
        
        self._counter = 0
       
        # Action client
        self._client = actionlib.SimpleActionClient(
            'move_bot_action_server', MoveBotAction)
        wait = self._client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        
        self.handle_inputs()
        rospy.spin()


    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")


    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        x = rospy.get_param("~x")
        y = rospy.get_param("~y")
        if x and y:
            self.move_to_goal(x, y)
        else:
            rospy.logerr("x or y is missing")
            rospy.on_shutdown(self.myhook)
            sys.exit(1)
 
 
    def move_to_goal(self, x, y):
        """
        Set up the goal and send it to the server

        Args:
            x (float): x coordinate
            y (float): y coordinate
        """
        rospy.loginfo("Moving the robot to ({}, {})".format(x, y))
        move_bot_action_goal = MoveBotGoal()
        move_bot_action_goal.x = x
        move_bot_action_goal.y = y

        self._client.send_goal(move_bot_action_goal,
                               done_cb=self.done_cb,
                               active_cb=self.active_cb,
                               feedback_cb=self.feedback_cb)
        


    def done_cb(self, status, result):
        """
        Called when the action is done

        Args:
            status (int): Return status by the server
            result (str): result message from the server
        """
        if status == 3:
            rospy.loginfo(result)


    def active_cb(self):
        rospy.loginfo(
            "Goal pose is now being processed by the Action Server...")


    def feedback_cb(self, msg):
        rospy.loginfo(msg)
        # self._counter += 1
        # if self._counter == 30:
        #     self._client.cancel_goal()
