

import rospy
from custom_msgs.srv import TakePicture
from service.service_server import ServiceServer


class ServiceClient(object):
    """Class which provides a Service server to handle Service client requests.
    """

    def __init__(self):
        rospy.init_node('service_client')
        # First make sure the server is working by waiting 5 seconds
        rospy.wait_for_service(ServiceServer.service_name(), 5)
        rospy.loginfo("Server available")
        self.handle_inputs()

    def myhook(self):
        """Function to call when shutting down a Node.
        """

        rospy.loginfo("shutdown time!")

    def handle_inputs(self):
        """Handle arguments passed to the command line.
        """

        if rospy.has_param('~picture'):
            picture = rospy.get_param("~picture")
            if picture:
                self.call_service(picture)
        else:
            rospy.logerr("Argument _picture:=<picture name> is missing")
            rospy.on_shutdown(self.myhook)

    def call_service(self, picture_name):
        """Send the service request

        Args:
            picture_name (str): Name of the picture to save the image on disk.
        """
        try:
            # Access the service
            handle = rospy.ServiceProxy(ServiceServer.service_name(), TakePicture)
            # call the service
            result = handle(picture_name)
            # display result received from the Server
            rospy.loginfo(result)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))
