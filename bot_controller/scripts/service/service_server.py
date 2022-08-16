import cv2
import rospy
from sensor_msgs.msg import Image
# ROS library that provides an interface between ROS and OpenCV
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.srv import TakePicture, TakePictureResponse


class ServiceServer(object):
    """
    Class which provides a Service server to handle Service client requests
    """

    def __init__(self):
        self._bridge = CvBridge()
        self._image_received = False
        self._image = None
        rospy.init_node('camera_server')
        # Subscribe to the turtlebot camera Topic
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_cb)
        # Callback to process client's request
        rospy.Service('take_picture_service', TakePicture, self.service_cb)
        rospy.loginfo("Server is ready to accept request")
        rospy.spin()


    def camera_cb(self, msg):
        """
        callback function to handle Messages published
        on the Topic /camera/rgb/image_raw

        Args:
            msg (Image): Image data
        """

        # Convert image to OpenCV format
        cv_image = None
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        self._image_received = True
        # numpy array
        self._image = cv_image


    def service_cb(self, request):
        """
        Callback function to process service client's request

        Args:
            request (str): Name of the picture sent by the client

        Returns:
            str: The result of the request
        """
        if self._image is not None:
            rospy.loginfo("Processing request")
            # Save an image
            final_image = "/tmp/"+str(request.picture_name)+'.jpg'
            # save image to a specified file
            write_status = cv2.imwrite(final_image, self._image)
            if write_status:
                rospy.loginfo("Image saved at {}".format(final_image))
                return TakePictureResponse('Picture saved')
            else:
                rospy.logerr("Image NOT saved")
                return TakePictureResponse('Picture Not saved')
        else:
            return TakePictureResponse('Picture Not saved')
