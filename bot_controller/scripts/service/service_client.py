

import rospy
from custom_msgs.srv import TakePicture


class ServiceClient(object):
    """
    Class which provides a Service server to handle Service client requests
    """

    def __init__(self):
        rospy.init_node('service_client')
        rospy.wait_for_service('take_picture_service')
        rospy.loginfo("Server available")
        self.handle_inputs()
        
    def myhook(self):
        """
        Function to call when shutting down a Node
        """
        rospy.loginfo("shutdown time!")
    
    def handle_inputs(self):
        """
        Handle arguments passed to the command line
        """

        if rospy.has_param('~picture'):
            picture = rospy.get_param("~picture")
            if picture:
                self.call_service(picture)
        else:
            rospy.logerr("Argument _picture:=<picture name> is missing")
            rospy.on_shutdown(self.myhook)
           


    def call_service(self, picture_name):
        try:
            # take_picture_service is the service name
            # TakePicture is the service type
            handle = rospy.ServiceProxy('take_picture_service', TakePicture)
            # call the service client
            result = handle(picture_name)
            rospy.loginfo(result)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))